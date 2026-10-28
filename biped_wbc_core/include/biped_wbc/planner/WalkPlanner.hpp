#pragma once
#include "Eigen/Dense"

#include "biped_wbc/common/data_type.hpp"
#include "biped_wbc/planner/LIPTaskPlanner.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <string>
#include <tsid/robots/robot-wrapper.hpp>

namespace biped_controllers {
    // phases of WalkPlanner, each Tswing long (LIP phase), except REST (until the next reset)
    enum class WalkPhase { INIT, STEP, STOP, REST };

    // Walking planner: CoM (LIPTaskPlanner) + swing foot references, both planned in the support
    // frame and returned in the world frame (TSID tasks).
    //
    // Support frame: on the ankle of the stance foot (left foot in double support), yaw only (x
    // forward along the foot, z up), taken from the forward kinematics of the estimated state once
    // per phase and then fixed until the next switch, so the references do not shake with the
    // estimate. At a switch the LIP state is re-expressed in the new frame.
    //
    // Timing: the CoM phase of LIPTaskPlanner drives the swing foot (time based switching): when the
    // phase wraps the swing foot has landed and becomes the stance foot. As walk_planner.cpp /
    // foot_placement.cpp of tsid_examples (commit 4de357c):
    //   init phase (both feet down): CP to the left ankle shifted by the CoM bias (com_bias_: the
    //   CoM keeps its place over the feet, e.g. its x ahead of the ankle), no swing
    //   steps, right foot first: the swing foot moves from liftoff to the foothold (step_length
    //   ahead, the stance width aside of the stance ankle) on a cycloid (zero velocity at both
    //   ends); the CP is driven to the end-of-step CP of the periodic gait, with a ZMP constant over
    //   the step and clamped to the stance sole
    //   stop (requestStop(), at the next phase end): both feet down, CP to the middle of the feet
    //   (shifted by the CoM bias)
    //   rest: double support, CoM at rest over the middle of the feet, until the next reset()
    // The LIP height is the CoM height and the stance width the ankle distance of the pose at
    // reset() (the robot walks at that height, on that width).
    class WalkPlanner
    {
        public:
            // robot: free-flyer robot model, owned by the caller (shared with the other modules, this
            // planner has its own pinocchio::Data); yaml: section "LIPPlanner" (LIP + step settings);
            // foot frames: the ankle frames of the support frames. Throws std::runtime_error if the
            // file, a value or a frame is invalid
            WalkPlanner(tsid::robots::RobotWrapper &robot, const std::string &CONFIG_YAML_PATH,
                        const std::string &left_foot_frame, const std::string &right_foot_frame);

            // restart from the estimated state q (both feet on the ground, at rest): support frame
            // on the left ankle, CoM at rest, LIP height = CoM height of q, init phase next
            void reset(const Eigen::VectorXd &q);

            // stop walking: the current phase ends as planned (a swing foot lands), then the stop
            // phase and rest. Cleared by reset()
            void requestStop() { this->stop_requested_ = true; }

            // forward speed [m/s] -> step length (clamped to [0, max_step_length]), used from the
            // next step on
            void setVelocityCommand(double vx);

            // main function run in controller update(): one planning cycle with the estimated state q
            // (forward kinematics only at a switch). Real-time (no allocation)
            void planWalking(const Eigen::VectorXd &q);

            // ---- references, world frame (valid after planWalking()) ----
            const Vector3d &getRefCoMPosition() const { return this->com_pos_; }
            const Vector3d &getRefCoMVelocity() const { return this->com_vel_; }
            const Vector3d &getRefCoMAcceleration() const { return this->com_acc_; }
            // swing foot ankle; in the init phase the right foot standing (velocity 0)
            const Vector3d &getRefSwingFootPosition() const { return this->swing_foot_pos_; }
            const Vector3d &getRefSwingFootVelocity() const { return this->swing_foot_vel_; }
            const Vector3d &getRefSwingFootAcceleration() const { return this->swing_foot_acc_; }
            // swing foot orientation: flat, same yaw as the support frame (no turning yet, B11)
            const Eigen::Matrix3d &getRefSwingFootRotation() const { return this->oMs_.rotation(); }
            Vector2d getRefZMP() const;  // world (x, y)
            Vector2d getRefCP() const;   // world (x, y)

            // ---- walking state ----
            WalkPhase getWalkPhase() const { return this->phase_; }
            // a foot is in the air (STEP): getSwingFoot() is that foot (0 left, 1 right); otherwise both
            // feet are on the ground
            bool isSwinging() const { return this->phase_ == WalkPhase::STEP; }
            int getSwingFoot() const { return this->left_stance_ ? 1 : 0; }
            bool isLeftStance() const { return this->left_stance_; }        // true: right foot swings
            bool isInitWalkingDone() const { return this->lip_.isInitWalkingDone(); }
            double getPhase() const { return this->lip_.getPhase(); }       // [0, 1)
            const pinocchio::SE3 &getSupportFrame() const { return this->oMs_; } // world <- support
            double getDt() const { return this->lip_.getDt(); }

        private:
            // support frame on the ankle of the given foot, from the forward kinematics of q (yaw only)
            pinocchio::SE3 supportFrame(const Eigen::VectorXd &q, bool left);
            // phase ended (LIP phase wrapped): next phase, with a support switch after a step
            void endOfPhase(const Eigen::VectorXd &q);
            // after a step: the landed foot is the stance foot, new support frame, LIP state into it,
            // liftoff position of the other foot
            void switchSupport(const Eigen::VectorXd &q, bool first_step);
            // both feet stay down: CP target in the middle of the feet, swing foot reference at rest
            void startStop();
            // swing foot reference at phase phi (support frame), from swing_start_pos_ to foothold_
            void computeSwingTrajectory(double phi);
            // references of this cycle from the support frame to the world frame
            void toWorld();

            tsid::robots::RobotWrapper &robot_; // robot model, owned by the caller
            pinocchio::Data data_;               // forward kinematics of this planner
            pinocchio::FrameIndex foot_id_[2];   // ankle frames [left, right]

            LIPTaskPlanner lip_;                 // CoM plan, in the support frame

            double T_{1.0}; // swing time (= LIP phase duration)
            double max_step_length_{0.3};
            double hip_width_{0.27};             // stance width [m]: ankle distance measured at reset()
            double sole_x_min_{-0.04};           // ZMP limits in single support [m], stance ankle frame
            double sole_x_max_{0.12};            // (defaults of tsid_examples walk_planner.h)
            double sole_y_half_{0.02};
            double step_height_{0.05};           // swing foot peak height [m] (as tsid_examples)
            double step_length_{0.0};            // from setVelocityCommand
            double ankle_height_{0.0};           // ankle above the ground [m] (reset): LIP zc -> ankle frame

            WalkPhase phase_ = WalkPhase::INIT;  // phase of this cycle
            bool stop_requested_ = false;        // requestStop(): stop at the next phase end
            Vector2d stop_target_ = Vector2d::Zero(); // CP target of STOP / REST: middle of the feet (support frame)
            Vector2d com_bias_ = Vector2d::Zero();    // CoM - middle of the feet at reset (support frame axes)
            bool left_stance_ = true;            // true: left foot stands, right foot swings
            double phi_prev_ = 0.0;              // LIP phase of the last cycle (detects the wrap)

            pinocchio::SE3 oMs_ = pinocchio::SE3::Identity(); // support frame (world <- support)

            // swing foot ankle (support frame); _l_ = local, the world copies are below
            Vector3d swing_start_pos_ = Vector3d::Zero(); // at liftoff (FK at the switch): start of the cycloid
            Vector3d foothold_ = Vector3d::Zero();        // landing (step_length, +-stance width, 0): end of the cycloid,
                                                          // z = 0: stance ankle height (CP target: periodic gait, planWalking)
            Vector3d swing_pos_l_ = Vector3d::Zero();     // reference of this cycle: cycloid + z bump at phase phi
            Vector3d swing_vel_l_ = Vector3d::Zero();     // d/dt = (d/dphi) / T (phi = t / T), 0 at both ends
            Vector3d swing_acc_l_ = Vector3d::Zero();     // d2/dt2 = (d2/dphi2) / T^2

            // references (world frame) for TSID, from the support frame oMs_ in toWorld(): positions
            // with the full transform, velocities / accelerations rotated only (frame fixed in a step)
            Vector3d com_pos_ = Vector3d::Zero();         // LIP (x, y), zc above the ground
            Vector3d com_vel_ = Vector3d::Zero();         // LIP w (cp - com), z = 0
            Vector3d com_acc_ = Vector3d::Zero();         // LIP w^2 (com - zmp), z = 0
            Vector3d swing_foot_pos_ = Vector3d::Zero();  // swing_pos_l_ in the world
            Vector3d swing_foot_vel_ = Vector3d::Zero();  // swing_vel_l_ rotated
            Vector3d swing_foot_acc_ = Vector3d::Zero();  // swing_acc_l_ rotated
    };
}
