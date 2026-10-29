#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>

#include "biped_wbc/common/utils.hpp"

#include "mjSimulator/LIPSimulator.hpp"

using Simulator::LIPSimulator;

// default files (source tree, set by CMake); override with the command line
#ifndef TASK_PLANNER_YAML
#define TASK_PLANNER_YAML "tsid_wbc.yaml"
#endif
#ifndef LIP_MODEL_XML
#define LIP_MODEL_XML "LIP_model.xml"
#endif

int main(int argc, char **argv)
{
    const std::string yaml_path = argc > 1 ? argv[1] : TASK_PLANNER_YAML;
    const std::string xml_path = argc > 2 ? argv[2] : LIP_MODEL_XML;

   

    LIPTaskPlanner lip_planner (yaml_path); 

    sim = std::make_unique<LIPSimulator>(xml_path); //simulator

    // ---- markers: mocap bodies of LIP_model.xml ----
    const int id_com = sim->MocapId("com"), id_shadow = sim->MocapId("com_shadow"), id_zmp = sim->MocapId("zmp"),
              id_cp = sim->MocapId("cp"), id_lf = sim->MocapId("left_foot"), id_rf = sim->MocapId("right_foot");

    // ---- trajectory history (one point every 10 ms, last 20 s) ----
    std::deque<Vector3d> com_hist, zmp_hist;
    const std::size_t max_hist = 2000;
    int ticks = 0;



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
        sim->AddConnector(mjGEOM_CAPSULE, 0.004, zmp, com, sim->black);                      // pendulum rod
        sim->AddConnector(mjGEOM_CAPSULE, 0.010, lf, com, sim->green);                       // left leg
        sim->AddConnector(mjGEOM_CAPSULE, 0.010, rf, com, sim->orange);                      // right leg
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
