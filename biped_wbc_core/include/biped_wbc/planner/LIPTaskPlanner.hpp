#pragma once
#include "Eigen/Dense"

#include "biped_wbc/common/data_type.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <tsid/robots/robot-wrapper.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace biped_controllers
{
    // This class wraps the Lineear Inverted Pendumlumn - based planner.
    // It receives the command from joystick and generates reference trajectory for
    // the walking locomotion task.
    //
    // C++ port of tsid_examples/python/WalkPlanner.py (+ LIP.py). Call planWalking() once per
    // control cycle (period dt from the yaml). Walking pattern:
    //   1. init phase (one Tswing, both feet on the ground): shift the capture point (CP) over the
    //      left foot
    //   2. steps, right foot first: the ZMP is placed so that the CP reaches the next foothold at the
    //      end of the step (Tswing); the swing foot follows a quadratic Bezier curve (x, z) from
    //      liftoff to the foothold, step_length ahead of the stance foot
    // No double support phase between the steps (as in the Python version).
    //
    // World frame: x forward, z up; the robot starts with its feet at y = +-hip_width / 2 (see
    // reset()). Outputs: CoM position / velocity / acceleration (TSID CoM task), foot position /
    // velocity / acceleration (TSID foot tasks), ZMP, CP, stance leg and phase.
    class LIPTaskPlanner
    {
        public:
            // constructor: pass an .yaml file which contains the setting parameters
            // (section "LIPPlanner"); throws std::runtime_error if the file is missing or invalid
            LIPTaskPlanner(const std::string &CONFIG_YAML_PATH);

            // LIP / capture point (CP) model in the horizontal plane (x, y), w = sqrt(g / zc):
            //   CoM:  d(com)/dt = w (cp - com)      the CoM follows the CP
            //   CP:   d(cp)/dt  = w (cp - zmp)      the CP diverges away from the ZMP

            // CoM velocity for the given CP and CoM
            Vector2d CoM_dynamics (const Vector2d &cp, const Vector2d &com) const;
            // CP velocity for the given ZMP and CP
            Vector2d CP_dynamics (const Vector2d &zmp, const Vector2d &cp) const;
            // one exact step (dt) of the internal CoM, driven by the CP; returns the new CoM
            Vector2d ComputeCoM (const Vector2d &cp);
            // one exact step (dt) of the internal CP, driven by the ZMP; returns the new CP
            Vector2d ComputeCP (const Vector2d &zmp);

            // ---- walking planner ----

            // restart the plan from the robot's current state (e.g. in WBCController::on_activate):
            // CoM (x, y) at rest (CP = CoM), feet positions in the world frame; back to the init phase
            void reset(const Vector2d &com, const Vector3d &left_foot, const Vector3d &right_foot);

            // forward velocity command [m/s] -> step length vx * Tswing, clipped to [0, max_step_length]
            // (takes effect at the next step)
            void setVelocityCommand(double vx);

            // one planning cycle: update the plan (ZMP, swing foot), then integrate the CP and the CoM
            void planWalking();

            // ---- reference outputs (valid after planWalking()) ----
            Vector3d getRefCoMPosition() const;     // (x, y, zc)
            Vector3d getRefCoMVelocity() const;     // (dx, dy, 0)
            Vector3d getRefCoMAcceleration() const; // (ddx, ddy, 0)
            const Vector3d &getRefLeftFootPosition() const { return this->left_foot_pos_; }
            const Vector3d &getRefRightFootPosition() const { return this->right_foot_pos_; }
            const Vector3d &getRefLeftFootVelocity() const { return this->left_foot_vel_; }
            const Vector3d &getRefRightFootVelocity() const { return this->right_foot_vel_; }
            const Vector3d &getRefLeftFootAcceleration() const { return this->left_foot_acc_; }
            const Vector3d &getRefRightFootAcceleration() const { return this->right_foot_acc_; }
            const Vector2d &getRefZMP() const { return this->zmp_d_; }
            const Vector2d &getRefCP() const { return this->cp_; }
            // DSt during the init phase, LSt (left stance, right swing) / RSt (right stance, left swing)
            LegState getStanceLeg() const;
            // phase of the current step / init phase in [0, 1) (ContactFusion: phi)
            double getPhase() const { return this->phi_com_; }
            bool isInitWalkingDone() const { return this->done_init_walking_; }
            // planner period [s] (1 / sampling_frequency of the yaml): call planWalking() at this rate
            double getDt() const { return this->dt_; }

        private:
            // swing foot helpers (WalkPlanner.start_swing / update_swing_foot)
            void startSwing(const Vector3d &swing_foot, const Vector3d &stance_foot);
            void updateSwingFoot(Vector3d &pos, Vector3d &vel, Vector3d &acc, double phi) const;
            void initWalking();
            // ZMP that brings the current CP to cp_d_ at the end of the step (recomputed every tick)
            Vector2d zmpForTargetCP() const;

            double T_{1.0};
            double dt_{0.002}; // default frequency is 500Hz
            double zc_{0.45};  // com height
            double max_step_length_{0.3};
            double hip_width_{0.27};
            double step_height_{0.1}; // swing foot peak height [m]
            std::vector<bool> command_stand_legs_{true, true};

            // LIP attributes
            const double g_ = 9.81;
            double w_ = std::sqrt(g_/zc_); // natural frequency [1/s]; recomputed in the constructor
                                           // after zc_ is read from the yaml

            // LIP state (horizontal, world frame), integrated by ComputeCoM() / ComputeCP()
            Vector2d com_ = Vector2d::Zero(); // CoM position
            Vector2d cp_ = Vector2d::Zero();  // capture point (CP)

            // walking state
            bool done_init_walking_ = false;
            bool left_stance_ = true;         // true: left foot stands, right foot swings
            double phi_com_ = 0.0;            // phase of the current step / init phase
            double step_length_ = 0.1;        // [m], from setVelocityCommand()
            Vector2d zmp_d_ = Vector2d::Zero(); // desired ZMP
            Vector2d cp_d_ = Vector2d::Zero();  // desired CP at the end of the step
            double zmp_hold_time_ = 0.02;       // [s] last part of a step with the ZMP held (b -> 1)

            // swing foot Bezier control points (offsets from the liftoff position), x and z
            Vector3d Px_d_ = Vector3d::Zero();
            Vector3d Pz_d_ = Vector3d::Zero();
            Vector3d swing_start_ = Vector3d::Zero(); // swing foot position at liftoff
            double foothold_x_ = 0.0;                 // landing x of the swing foot

            // feet references (world frame)
            Vector3d left_foot_pos_ = Vector3d::Zero();
            Vector3d right_foot_pos_ = Vector3d::Zero();
            Vector3d left_foot_vel_ = Vector3d::Zero();
            Vector3d right_foot_vel_ = Vector3d::Zero();
            Vector3d left_foot_acc_ = Vector3d::Zero();
            Vector3d right_foot_acc_ = Vector3d::Zero();
    };

} //namespace biped_controllers
