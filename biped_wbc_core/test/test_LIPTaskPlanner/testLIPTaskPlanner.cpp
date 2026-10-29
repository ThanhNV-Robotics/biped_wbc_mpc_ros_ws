// Visual test of WalkPlanner (LIPTaskPlanner CoM + swing foot) in MuJoCo (no physics): runs the
// planner in real time and shows the same elements as tsid_examples/python/SimDraw.py:
//   CoM (blue) with its ground shadow and vertical projection, ZMP (crimson) with the foot sole
//   outline, capture point (red cross), left / right foot (green / orange), pendulum rod ZMP -> CoM,
//   legs feet -> CoM, CoM and ZMP trajectories. The camera follows the CoM.
// WalkPlanner takes the landed foot from the forward kinematics of the robot state at each switch,
// so the test runs an ideal robot: each cycle the leg IK puts the stance ankle on the support frame
// and the swing ankle on its reference, base above the planned CoM.
// The window, camera, keyboard and drawing are in Simulator::LIPSimulator (test/mjSimulator).
//
// Usage:
//   ros2 run biped_wbc_core test_lip_task_planner [tsid_wbc.yaml] [LIP_model.xml] [robot.urdf]
// Keys: space = pause / run, up / down = forward velocity command +- 0.05 m/s, backspace = restart.
// Mouse: left drag = rotate, right drag = move, scroll = zoom.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>

#include "biped_wbc/common/utils.hpp"
#include "biped_wbc/planner/WalkPlanner.hpp"
#include "mjSimulator/LIPSimulator.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>

using biped_controllers::WalkPlanner;
using Simulator::LIPSimulator;

// default files (source tree, set by CMake); override with the command line
#ifndef TASK_PLANNER_YAML
#define TASK_PLANNER_YAML "tsid_wbc.yaml"
#endif
#ifndef LIP_MODEL_XML
#define LIP_MODEL_XML "LIP_model.xml"
#endif
#ifndef ROBOT_URDF
#define ROBOT_URDF "v2_biped_robot_12dof.urdf"
#endif

int main(int argc, char **argv)
{
    const std::string yaml_path = argc > 1 ? argv[1] : TASK_PLANNER_YAML;
    const std::string xml_path = argc > 2 ? argv[2] : LIP_MODEL_XML;
    const std::string urdf_path = argc > 3 ? argv[3] : ROBOT_URDF;
    const std::string feet[2] = {"left_ankle_pitch_link", "right_ankle_pitch_link"};

    // ---- robot model, planner, MuJoCo viewer (markers only) ----
    std::unique_ptr<tsid::robots::RobotWrapper> robot;
    std::unique_ptr<WalkPlanner> planner;
    std::unique_ptr<LIPSimulator> sim;
    try
    {
        robot = std::make_unique<tsid::robots::RobotWrapper>(urdf_path, std::vector<std::string>{},
                                                             pinocchio::JointModelFreeFlyer(), false);
        planner = std::make_unique<WalkPlanner>(*robot, yaml_path, feet[0], feet[1]);
        sim = std::make_unique<LIPSimulator>(xml_path);
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
    const double dt = planner->getDt();
    planner->setVelocityCommand(sim->GetVelocityCommand());

    // ---- ideal robot: start pose, feet hip_width apart, base kBaseHeight above the ankles ----
    const pinocchio::Model &model = robot->model();
    pinocchio::Data data(model);
    const double kHipWidth = 0.27, kBaseHeight = 0.72, kAnkleHeight = 0.05; // [m]
    Eigen::VectorXd q0 = pinocchio::neutral(model);
    q0.tail(robot->na()) = utils::computeIntialStandConfig(model, data, kHipWidth, kBaseHeight);
    q0(2) = kBaseHeight + kAnkleHeight; // ankles kAnkleHeight above the ground (z = 0)
    Eigen::VectorXd q = q0;
    planner->reset(q);
    int ik_failures = 0;

    // leg IK: stance ankle on the support frame, swing ankle on its reference, base (upright) above
    // the planned CoM at its start height -> q (kept if the IK does not converge)
    const auto trackPlan = [&]() {
        const pinocchio::SE3 oMb(Eigen::Matrix3d::Identity(),
                                 Vector3d(planner->getRefCoMPosition().x(), planner->getRefCoMPosition().y(), q0(2)));
        const pinocchio::SE3 oMstance = planner->getSupportFrame();
        const pinocchio::SE3 oMswing(planner->getRefSwingFootRotation(), planner->getRefSwingFootPosition());
        const bool left = planner->isLeftStance();
        const pinocchio::SE3 bMl = oMb.actInv(left ? oMstance : oMswing);
        const pinocchio::SE3 bMr = oMb.actInv(left ? oMswing : oMstance);
        const utils::IkRes ik = utils::computeIK_Leg(model, data, bMl.rotation(), bMl.translation(),
                                                     bMr.rotation(), bMr.translation());
        if (ik.status != 0)
        {
            if (ik_failures++ == 0) std::fprintf(stderr, "leg IK did not converge (error %.2e): pose kept\n", ik.err.norm());
            return;
        }
        q.head<3>() = oMb.translation();
        q.tail(robot->na()) = ik.jointPosRes;
    };

    // ---- markers: mocap bodies of LIP_model.xml ----
    const int id_com = sim->MocapId("com"), id_shadow = sim->MocapId("com_shadow"), id_zmp = sim->MocapId("zmp"),
              id_cp = sim->MocapId("cp"), id_lf = sim->MocapId("left_foot"), id_rf = sim->MocapId("right_foot");

    // ---- trajectory history (one point every 10 ms, last 20 s) ----
    std::deque<Vector3d> com_hist, zmp_hist;
    const std::size_t max_hist = 2000;
    int ticks = 0;

    // colors (as SimDraw.py)
    const float black[4] = {0.f, 0.f, 0.f, 0.5f};
    const float green[4] = {0.13f, 0.55f, 0.13f, 0.85f};
    const float orange[4] = {1.f, 0.55f, 0.f, 0.85f};
    const float blue[4] = {0.f, 0.f, 1.f, 0.4f};
    const float com_traj[4] = {0.f, 0.f, 1.f, 0.7f};
    const float zmp_traj[4] = {1.f, 0.f, 0.f, 0.7f};

    double sim_time = 0.0;
    auto wall_start = std::chrono::steady_clock::now();

    while (sim->IsOpen())
    {
        // ---- restart / velocity command from the keyboard ----
        if (sim->ConsumeRestart())
        {
            q = q0;
            planner->reset(q);
            com_hist.clear();
            zmp_hist.clear();
        }
        planner->setVelocityCommand(sim->GetVelocityCommand());
        const bool paused = sim->IsPaused();

        // ---- run the planner in real time: as many dt steps as the wall clock advanced ----
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
        if (paused)
        {
            sim_time = wall; // do not catch up after a pause
        }
        while (!paused && sim_time + dt <= wall)
        {
            planner->planWalking(q);
            trackPlan();
            sim_time += dt;
            if (++ticks % std::max(1, static_cast<int>(0.01 / dt)) == 0)
            {
                com_hist.push_back(planner->getRefCoMPosition());
                const Vector2d z = planner->getRefZMP();
                zmp_hist.push_back(Vector3d(z.x(), z.y(), 0.0));
                if (com_hist.size() > max_hist) com_hist.pop_front();
                if (zmp_hist.size() > max_hist) zmp_hist.pop_front();
            }
        }

        // ---- planner outputs -> markers ----
        const Vector3d com = planner->getRefCoMPosition();
        const Vector2d zmp2 = planner->getRefZMP(), cp2 = planner->getRefCP();
        const Vector3d zmp(zmp2.x(), zmp2.y(), 0.0), cp(cp2.x(), cp2.y(), 0.0);
        // feet on the ground: ankles of the ideal robot, kAnkleHeight lower
        pinocchio::framesForwardKinematics(model, data, q);
        const Vector3d lf = data.oMf[model.getFrameId(feet[0])].translation() - Vector3d(0.0, 0.0, kAnkleHeight);
        const Vector3d rf = data.oMf[model.getFrameId(feet[1])].translation() - Vector3d(0.0, 0.0, kAnkleHeight);
        sim->SetMocap(id_com, com);
        sim->SetMocap(id_shadow, Vector3d(com.x(), com.y(), 0.001));
        sim->SetMocap(id_zmp, Vector3d(zmp.x(), zmp.y(), 0.002));
        sim->SetMocap(id_cp, Vector3d(cp.x(), cp.y(), 0.003));
        sim->SetMocap(id_lf, lf);
        sim->SetMocap(id_rf, rf);

        // camera follows the CoM, low-pass filtered (smooths the lateral sway)
        sim->FollowTarget(com.x(), com.y());

        // ---- render: model geoms, then the lines of SimDraw.py ----
        sim->BeginFrame();
        sim->AddConnector(mjGEOM_CAPSULE, 0.004, zmp, com, black);                      // pendulum rod
        sim->AddConnector(mjGEOM_CAPSULE, 0.010, lf, com, green);                       // left leg
        sim->AddConnector(mjGEOM_CAPSULE, 0.010, rf, com, orange);                      // right leg
        sim->AddConnector(mjGEOM_LINE, 1.0, Vector3d(com.x(), com.y(), 0.0), com, blue); // projection
        sim->AddPolyline(com_hist, 2.0, com_traj);                                      // CoM trajectory
        sim->AddPolyline(zmp_hist, 1.5, zmp_traj);                                      // ZMP trajectory

        // ---- text overlay: time, stance leg, phase, command; then swap and poll events ----
        const char *stance[] = {"left (right swings)", "right (left swings)", "double (init)"};
        char labels[200], values[200];
        std::snprintf(labels, sizeof(labels), "time\nstance\nphase\nvx command\n%s", paused ? "PAUSED" : "");
        const int stance_id = !planner->isInitWalkingDone() ? 2 : (planner->isLeftStance() ? 0 : 1);
        std::snprintf(values, sizeof(values), "%.2f s\n%s\n%.2f\n%.2f m/s\n", sim_time,
                      stance[stance_id], planner->getPhase(), sim->GetVelocityCommand());
        sim->EndFrame(labels, values);
    }

    // the simulator frees the scene, the window and the MuJoCo model
    return 0;
}
