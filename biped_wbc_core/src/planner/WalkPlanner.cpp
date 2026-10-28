#include "biped_wbc/planner/WalkPlanner.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace biped_controllers {
    // Reads the step settings from the "LIPPlanner" section of the yaml (the LIP keys are read by
    // LIPTaskPlanner), e.g.
    //   max_step_length: 0.3   # [m]
    //   step_height: 0.05      # [m] swing foot peak height
    //   sole_x_min: -0.04      # [m] ZMP limits in single support, stance ankle frame
    //   sole_x_max: 0.12
    //   sole_y_half: 0.02
    // A key that is missing keeps its default from the header. The stance width is measured at
    // reset() (the hip_width key is not used).
    WalkPlanner::WalkPlanner(tsid::robots::RobotWrapper &robot, const std::string &CONFIG_YAML_PATH,
                             const std::string &left_foot_frame, const std::string &right_foot_frame)
        : robot_(robot), data_(robot.model()), lip_(CONFIG_YAML_PATH)
    {
        // ankle frames of the support frames
        const pinocchio::Model &model = this->robot_.model();
        const std::string frames[2] = {left_foot_frame, right_foot_frame};
        for (int i = 0; i < 2; ++i)
        {
            if (!model.existFrame(frames[i]))
            {
                throw std::runtime_error("WalkPlanner: no frame '" + frames[i] + "' in the robot model");
            }
            this->foot_id_[i] = model.getFrameId(frames[i]);
        }

        // step settings (the file was already checked by LIPTaskPlanner)
        const YAML::Node cfg = YAML::LoadFile(CONFIG_YAML_PATH)["LIPPlanner"];
        const auto read = [&](const char *key, double &value) {
            if (!cfg[key]) return;
            try
            {
                value = cfg[key].as<double>();
            }
            catch (const YAML::Exception &e)
            {
                throw std::runtime_error(std::string("WalkPlanner: '") + key + "' in '" +
                                         CONFIG_YAML_PATH + "' is not a number: " + e.what());
            }
        };
        read("max_step_length", this->max_step_length_);
        read("step_height", this->step_height_);
        read("sole_x_min", this->sole_x_min_);
        read("sole_x_max", this->sole_x_max_);
        read("sole_y_half", this->sole_y_half_);
        if (this->max_step_length_ < 0.0 || this->step_height_ < 0.0 || this->sole_x_min_ > this->sole_x_max_ ||
            this->sole_y_half_ < 0.0)
        {
            throw std::runtime_error("WalkPlanner: max_step_length, step_height and sole_y_half must be >= 0, "
                                     "sole_x_min <= sole_x_max (file '" + CONFIG_YAML_PATH + "')");
        }

        // one phase of the LIP = one swing
        this->T_ = this->lip_.getTswing();
    }

    // support frame on an ankle: its position, yaw of its orientation only (z up)
    pinocchio::SE3 WalkPlanner::supportFrame(const Eigen::VectorXd &q, bool left)
    {
        // ankle frames of q (both: the caller may also need the other foot)
        const pinocchio::Model &model = this->robot_.model();
        pinocchio::forwardKinematics(model, this->data_, q);
        pinocchio::updateFramePlacement(model, this->data_, this->foot_id_[0]);
        pinocchio::updateFramePlacement(model, this->data_, this->foot_id_[1]);
        const pinocchio::SE3 &oMa = this->data_.oMf[this->foot_id_[left ? 0 : 1]];

        // yaw only: x forward along the foot, z up
        const double yaw = std::atan2(oMa.rotation()(1, 0), oMa.rotation()(0, 0));
        return pinocchio::SE3(Eigen::AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix(), oMa.translation());
    }

    void WalkPlanner::reset(const Eigen::VectorXd &q)
    {
        // double support: support frame on the left ankle; ankle height above the ground (z = 0 in
        // the estimator), so that the CoM is zc above the ground
        this->left_stance_ = true;
        this->oMs_ = this->supportFrame(q, true);
        this->ankle_height_ = this->oMs_.translation().z();

        // right foot standing where it is (support frame), at rest
        this->swing_start_pos_ = this->oMs_.actInv(this->data_.oMf[this->foot_id_[1]].translation());
        this->foothold_ = this->swing_start_pos_;
        this->swing_pos_l_ = this->swing_start_pos_;
        this->swing_vel_l_.setZero();
        this->swing_acc_l_.setZero();

        // CoM at rest in the support frame, LIP at the CoM height above the ground, init phase next
        const Vector3d com = this->oMs_.actInv(pinocchio::centerOfMass(this->robot_.model(), this->data_, q));
        this->lip_.setComHeight(com.z() + this->ankle_height_);
        this->com_bias_ = com.head<2>() - 0.5 * this->swing_start_pos_.head<2>(); // middle: half the right ankle

        // stance width: distance of the ankles now (the footholds and CP targets lie on the feet)
        this->hip_width_ = this->swing_start_pos_.head<2>().norm();
        this->lip_.reset(com.head<2>());
        this->phi_prev_ = 0.0;
        this->phase_ = WalkPhase::INIT;
        this->stop_requested_ = false;

        this->toWorld();
    }

    void WalkPlanner::setVelocityCommand(double vx)
    {
        this->step_length_ = std::clamp(vx * this->T_, 0.0, this->max_step_length_);
    }

    // main function run in control update()
    void WalkPlanner::planWalking(const Eigen::VectorXd &q)
    {
        // CP target of this phase (support frame) and ZMP limits, as walk_planner.cpp of
        // tsid_examples (commit 4de357c):
        //   init: the stance ankle shifted by the CoM bias (the CoM keeps its x), double support
        //   step: end-of-step CP of the periodic gait with a constant ZMP at the stance ankle,
        //         sagittal s b / (b - 1), lateral y_f b / (b + 1) (b = e^{w T}, y_f = foothold y);
        //         ZMP on the stance sole
        //   stop / rest: middle of the feet shifted by the CoM bias, double support
        Vector2d cp_target = this->com_bias_;
        Vector2d zmp_min = Vector2d::Constant(-1e9), zmp_max = Vector2d::Constant(1e9);
        if (this->phase_ == WalkPhase::STEP)
        {
            const double b = std::exp(this->lip_.getOmega() * this->T_);
            cp_target << this->foothold_.x() * b / (b - 1.0), this->foothold_.y() * b / (b + 1.0);
            zmp_min << this->sole_x_min_, -this->sole_y_half_;
            zmp_max << this->sole_x_max_, this->sole_y_half_;
        }
        else if (this->phase_ == WalkPhase::STOP || this->phase_ == WalkPhase::REST)
        {
            cp_target += this->stop_target_;
        }

        // CoM: one LIP cycle
        this->lip_.planWalking(cp_target, zmp_min, zmp_max);

        // phase wrapped (time based): next phase
        if (this->lip_.getPhase() < this->phi_prev_)
        {
            this->endOfPhase(q);
        }
        this->phi_prev_ = this->lip_.getPhase();

        // swing foot of this cycle (both feet stand in the other phases)
        if (this->phase_ == WalkPhase::STEP)
        {
            this->computeSwingTrajectory(this->lip_.getPhase());
        }

        this->toWorld();
    }

    // INIT -> STEP (right foot first) or STOP; STEP -> STEP (support switch) or STOP (after the
    // switch: the landed foot stays down); STOP -> REST; REST stays
    void WalkPlanner::endOfPhase(const Eigen::VectorXd &q)
    {
        switch (this->phase_)
        {
        case WalkPhase::INIT:
            if (this->stop_requested_)
            {
                this->startStop(); // no step: the right foot is still where reset() measured it
            }
            else
            {
                this->switchSupport(q, true);
                this->phase_ = WalkPhase::STEP;
            }
            break;
        case WalkPhase::STEP:
            this->switchSupport(q, false);
            if (this->stop_requested_)
            {
                this->startStop();
            }
            break;
        case WalkPhase::STOP:
            this->phase_ = WalkPhase::REST;
            break;
        case WalkPhase::REST:
            break;
        }
    }

    void WalkPlanner::startStop()
    {
        // the other foot stands at its liftoff position (support frame), CP between the ankles
        this->phase_ = WalkPhase::STOP;
        this->stop_target_ = 0.5 * this->swing_start_pos_.head<2>();
        this->swing_pos_l_ = this->swing_start_pos_;
        this->swing_vel_l_.setZero();
        this->swing_acc_l_.setZero();
    }

    // first_step: end of the init phase, the left foot stays the stance foot and the right one swings;
    // otherwise the landed foot becomes the stance foot
    void WalkPlanner::switchSupport(const Eigen::VectorXd &q, bool first_step)
    {
        // new stance foot and its support frame from the estimated state
        if (!first_step)
        {
            this->left_stance_ = !this->left_stance_;
        }
        const pinocchio::SE3 oMs_old = this->oMs_;
        this->oMs_ = this->supportFrame(q, this->left_stance_);

        // LIP state into the new frame (both frames yaw only: the planar part is exact)
        const pinocchio::SE3 sMs_old = this->oMs_.actInv(oMs_old);
        this->lip_.transformState(sMs_old.rotation().topLeftCorner<2, 2>(), sMs_old.translation().head<2>());

        // swing foot: lifts off where it is now, lands step_length ahead and hip_width aside of
        // the stance ankle, at its height
        const int swing = this->left_stance_ ? 1 : 0;
        this->swing_start_pos_ = this->oMs_.actInv(this->data_.oMf[this->foot_id_[swing]].translation());
        this->foothold_ << this->step_length_, this->left_stance_ ? -this->hip_width_ : this->hip_width_, 0.0;
    }

    // swing foot at phase phi (support frame), as foot_placement.cpp of tsid_examples: cycloid from
    // liftoff to the foothold, zero velocity at both ends, plus a z bump of step_height at phi = 0.5
    //   c(phi) = (2 pi phi - sin(2 pi phi)) / (2 pi),  bump(phi) = h (1 - cos(2 pi phi)) / 2
    void WalkPlanner::computeSwingTrajectory(double phi)
    {
        // cycloid and bump with their derivatives w.r.t. phi
        const double w = 2.0 * M_PI;
        const double c = (w * phi - std::sin(w * phi)) / w;
        const double dc = 1.0 - std::cos(w * phi);
        const double ddc = w * std::sin(w * phi);
        const double h = this->step_height_;
        const double bump = 0.5 * h * (1.0 - std::cos(w * phi));
        const double dbump = 0.5 * h * w * std::sin(w * phi);
        const double ddbump = 0.5 * h * w * w * std::cos(w * phi);

        // position, and the time derivatives (dphi/dt = 1 / T)
        const Vector3d delta = this->foothold_ - this->swing_start_pos_;
        const double T2 = this->T_ * this->T_;
        this->swing_pos_l_ = this->swing_start_pos_ + c * delta;
        this->swing_pos_l_.z() += bump;
        this->swing_vel_l_ = (dc / this->T_) * delta;
        this->swing_vel_l_.z() += dbump / this->T_;
        this->swing_acc_l_ = (ddc / T2) * delta;
        this->swing_acc_l_.z() += ddbump / T2;
    }

    // support frame -> world: positions with the full transform, velocities / accelerations with the
    // rotation only (the frame is fixed during a phase)
    void WalkPlanner::toWorld()
    {
        const Eigen::Matrix3d &R = this->oMs_.rotation();

        // CoM: LIP (x, y) at zc above the ground = zc - ankle_height above the stance ankle
        Vector3d com_l = this->lip_.getRefCoMPosition();
        com_l.z() -= this->ankle_height_;
        this->com_pos_ = this->oMs_.act(com_l);
        this->com_vel_.noalias() = R * this->lip_.getRefCoMVelocity();
        this->com_acc_.noalias() = R * this->lip_.getRefCoMAcceleration();

        // swing foot ankle
        this->swing_foot_pos_ = this->oMs_.act(this->swing_pos_l_);
        this->swing_foot_vel_.noalias() = R * this->swing_vel_l_;
        this->swing_foot_acc_.noalias() = R * this->swing_acc_l_;
    }

    Vector2d WalkPlanner::getRefZMP() const
    {
        const Vector2d &zmp = this->lip_.getRefZMP();
        return this->oMs_.act(Vector3d(zmp.x(), zmp.y(), 0.0)).head<2>();
    }

    Vector2d WalkPlanner::getRefCP() const
    {
        const Vector2d &cp = this->lip_.getRefCP();
        return this->oMs_.act(Vector3d(cp.x(), cp.y(), 0.0)).head<2>();
    }
}
