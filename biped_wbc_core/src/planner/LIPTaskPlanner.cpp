#include "biped_wbc/planner/LIPTaskPlanner.hpp"

#include <stdexcept>

namespace biped_controllers

{
    // Reads the planner settings from the "LIPPlanner" section of a yaml file, e.g.
    //   LIPPlanner:
    //     Tswing: 1.0               # [s] swing (step) duration
    //     sampling_frequence: 500   # [Hz] planner rate -> dt = 1 / f
    //     com_height: 0.45          # [m] CoM height of the LIP model
    //     max_step_length: 0.3      # [m]
    // A key that is missing keeps its default from the header. A missing file / section or an
    // invalid value throws std::runtime_error (a constructor cannot return an error code); the
    // caller, e.g. WBCController::on_configure(), catches it and fails cleanly.
    LIPTaskPlanner::LIPTaskPlanner(const std::string &CONFIG_YAML_PATH)
    {
        // load the file; YAML::LoadFile throws YAML::BadFile if it cannot be opened
        YAML::Node root;
        try
        {
            root = YAML::LoadFile(CONFIG_YAML_PATH);
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("LIPTaskPlanner: cannot load '" + CONFIG_YAML_PATH + "': " + e.what());
        }

        // all settings are under the "LIPPlanner" key
        const YAML::Node cfg = root["LIPPlanner"];
        if (!cfg || !cfg.IsMap())
        {
            throw std::runtime_error("LIPTaskPlanner: no 'LIPPlanner' section in '" + CONFIG_YAML_PATH + "'");
        }

        // read a value if the key exists, otherwise keep the default; a value that is not a number
        // (e.g. "0.3=") throws with the key name
        const auto read = [&](const char *key, double &value) {
            if (!cfg[key]) return;
            try
            {
                value = cfg[key].as<double>();
            }
            catch (const YAML::Exception &e)
            {
                throw std::runtime_error(std::string("LIPTaskPlanner: '") + key + "' in '" +
                                         CONFIG_YAML_PATH + "' is not a number: " + e.what());
            }
        };

        // swing duration and LIP CoM height
        read("Tswing", this->T_);
        read("com_height", this->zc_);
        read("max_step_length", this->max_step_length_);
        read("hip_width", this->hip_width_);
        read("step_height", this->step_height_); // optional, default 0.1 m

        // the yaml gives the rate, the planner works with the sampling time
        double frequency = 1.0 / this->dt_;
        read("sampling_frequency", frequency);

        // check the values before using them (a zero rate or height would divide by zero later)
        if (this->T_ <= 0.0 || this->zc_ <= 0.0 || frequency <= 0.0 || this->max_step_length_ < 0.0)
        {
            throw std::runtime_error("LIPTaskPlanner: Tswing, com_height and sampling_frequency must be > 0, "
                                     "max_step_length >= 0 (file '" + CONFIG_YAML_PATH + "')");
        }
        this->dt_ = 1.0 / frequency;

        // LIP natural frequency from the CoM height read above (the member initializer used the
        // default zc_)
        this->w_ = std::sqrt(this->g_ / this->zc_);

        // start like WalkPlanner.py: CoM at the origin, feet at y = +-hip_width / 2; call reset()
        // with the robot's state before walking
        this->reset(Vector2d::Zero(), Vector3d(0.0, 0.5 * this->hip_width_, 0.0),
                    Vector3d(0.0, -0.5 * this->hip_width_, 0.0));
    }

    // CoM dynamics of the LIP: the CoM is pulled toward the capture point
    Vector2d LIPTaskPlanner::CoM_dynamics(const Vector2d &cp, const Vector2d &com) const
    {
        return this->w_ * (cp - com);
    }

    // CP dynamics: the capture point moves away from the ZMP (unstable part of the LIP)
    Vector2d LIPTaskPlanner::CP_dynamics(const Vector2d &zmp, const Vector2d &cp) const
    {
        return this->w_ * (cp - zmp);
    }

    // integrate the internal CoM one step with the given CP, exactly for a CP constant over dt:
    //   d(com)/dt = w (cp - com)  ->  com(t + dt) = cp + e^{-w dt} (com - cp)
    // (explicit Euler lags behind the exponential and makes the plan drift, see ComputeCP)
    Vector2d LIPTaskPlanner::ComputeCoM(const Vector2d &cp)
    {
        this->com_ = cp + std::exp(-this->w_ * this->dt_) * (this->com_ - cp);
        return this->com_; // returned by value: a copy, no aliasing of the member
    }

    // integrate the internal CP one step with the given ZMP, exactly for a ZMP constant over dt:
    //   d(cp)/dt = w (cp - zmp)  ->  cp(t + dt) = zmp + e^{w dt} (cp - zmp)
    // Explicit Euler grows by (1 + w dt) per step instead of e^{w dt}: over one step (T = 1 s,
    // 500 Hz) 2 % short of e^{wT}, so the CP missed the foothold by ~3 mm per step and the
    // error added up in x.
    Vector2d LIPTaskPlanner::ComputeCP(const Vector2d &zmp)
    {
        this->cp_ = zmp + std::exp(this->w_ * this->dt_) * (this->cp_ - zmp); // analytical solution of the Captured Point dynamics
        return this->cp_;
    }

    // ---------------------------------------------------------------------
    // Walking planner (port of tsid_examples/python/WalkPlanner.py)
    // ---------------------------------------------------------------------

    // quadratic Bezier curve with control points P = (P0, P1, P2) at s in [0, 1] (bezier_1D with
    // 3 points), and its first / second derivative with respect to s
    static double bezier2(const Vector3d &P, double s)
    {
        return (1.0 - s) * (1.0 - s) * P(0) + 2.0 * (1.0 - s) * s * P(1) + s * s * P(2);
    }
    static double bezier2_ds(const Vector3d &P, double s)
    {
        return 2.0 * (1.0 - s) * (P(1) - P(0)) + 2.0 * s * (P(2) - P(1));
    }
    static double bezier2_dds(const Vector3d &P)
    {
        return 2.0 * (P(2) - 2.0 * P(1) + P(0));
    }

    // restart from the robot's state: CoM at rest (CP = CoM), feet where they are, init phase next
    void LIPTaskPlanner::reset(const Vector2d &com, const Vector3d &left_foot, const Vector3d &right_foot)
    {
        // LIP state at rest
        this->com_ = com;
        this->cp_ = com;
        this->cp_d_ = com;
        this->zmp_d_ = com;

        // feet standing still
        this->left_foot_pos_ = left_foot;
        this->right_foot_pos_ = right_foot;
        this->left_foot_vel_.setZero();
        this->right_foot_vel_.setZero();
        this->left_foot_acc_.setZero();
        this->right_foot_acc_.setZero();

        // walking state: init phase first, then the right foot swings (left stance)
        this->done_init_walking_ = false;
        this->left_stance_ = true;
        this->phi_com_ = 0.0;
        this->swing_start_.setZero();
        this->foothold_x_ = 0.0;
        this->Px_d_.setZero();
        this->Pz_d_.setZero();
    }

    // step length from the forward velocity command (WalkPlanner.computeStepLength)
    void LIPTaskPlanner::setVelocityCommand(double vx)
    {
        this->step_length_ = std::clamp(vx * this->T_, 0.0, this->max_step_length_);
    }

    // ZMP that brings the CP from where it is now to cp_d_ at the end of the step.
    // CP solution with a constant ZMP p over the remaining time t_r: cp(t_r) = p + e^{w t_r} (cp - p).
    // Solved for p so that cp(t_r) = cp_d:  p = (cp_d - b cp) / (1 - b), b = e^{w t_r}
    // Recomputed every tick from the current CP (closed loop): any CP error, from the integration
    // or later from the robot's estimated CP, is corrected within the same step.
    Vector2d LIPTaskPlanner::zmpForTargetCP() const
    {
        const double t_remaining = this->T_ * (1.0 - this->phi_com_);
        // near the end of the step b -> 1 and the formula is singular: keep the last ZMP
        if (t_remaining < this->zmp_hold_time_) return this->zmp_d_;
        const double b = std::exp(this->w_ * t_remaining);
        return (this->cp_d_ - b * this->cp_) / (1.0 - b);
    }

    // latch the liftoff position; the foot lands step_length ahead of the stance foot
    // (WalkPlanner.start_swing)
    void LIPTaskPlanner::startSwing(const Vector3d &swing_foot, const Vector3d &stance_foot)
    {
        this->swing_start_ = swing_foot;
        this->foothold_x_ = stance_foot.x() + this->step_length_; // landing x, also the target CP x
        const double dx = this->foothold_x_ - this->swing_start_.x();
        // Bezier control points as offsets from liftoff: x moves dx, z peaks at step_height
        // (a quadratic Bezier peaks at half its middle point, so the middle point is 2 h)
        this->Px_d_ << 0.0, 0.5 * dx, dx;
        this->Pz_d_ << 0.0, 2.0 * this->step_height_, 0.0;
    }

    // swing foot at phase phi: position from the Bezier offsets (assigned, never accumulated),
    // velocity / acceleration from its derivatives (d/dt = d/dphi / T); y stays constant
    // (WalkPlanner.update_swing_foot)
    void LIPTaskPlanner::updateSwingFoot(Vector3d &pos, Vector3d &vel, Vector3d &acc, double phi) const
    {
        const double T2 = this->T_ * this->T_;
        pos.x() = this->swing_start_.x() + bezier2(this->Px_d_, phi);
        pos.z() = this->swing_start_.z() + bezier2(this->Pz_d_, phi);
        vel << bezier2_ds(this->Px_d_, phi) / this->T_, 0.0, bezier2_ds(this->Pz_d_, phi) / this->T_;
        acc << bezier2_dds(this->Px_d_) / T2, 0.0, bezier2_dds(this->Pz_d_) / T2;
    }

    // init phase: shift the CP over the left foot in one Tswing (WalkPlanner.init_walking)
    void LIPTaskPlanner::initWalking()
    {
        // ramp up the phase
        this->phi_com_ += this->dt_ / this->T_;

        // target: CP over the left foot (x, y)
        this->cp_d_ = this->left_foot_pos_.head<2>();

        if (this->phi_com_ >= 1.0)
        {
            // move the ZMP onto the left foot and start the first step: right foot swings
            this->zmp_d_ = this->left_foot_pos_.head<2>();
            this->phi_com_ = 0.0;
            this->done_init_walking_ = true;
            this->startSwing(this->right_foot_pos_, this->left_foot_pos_);
        }
        else
        {
            this->zmp_d_ = this->zmpForTargetCP();
        }
    }

    // one planning cycle (WalkPlanner.plan_walking)
    void LIPTaskPlanner::planWalking()
    {
        // feet are still unless they swing (set below)
        this->left_foot_vel_.setZero();
        this->right_foot_vel_.setZero();
        this->left_foot_acc_.setZero();
        this->right_foot_acc_.setZero();

        //------------------------------------------
        // Init walking: move the CoM to the left side
        //------------------------------------------
        if (!this->done_init_walking_)
        {
            this->initWalking();
        }
        //------------------------------------------
        // Walking: alternate stance / swing legs
        //------------------------------------------
        else
        {
            this->phi_com_ += this->dt_ / this->T_;

            // the stance and swing feet of this step
            Vector3d &stance_pos = this->left_stance_ ? this->left_foot_pos_ : this->right_foot_pos_;
            Vector3d &swing_pos = this->left_stance_ ? this->right_foot_pos_ : this->left_foot_pos_;
            Vector3d &swing_vel = this->left_stance_ ? this->right_foot_vel_ : this->left_foot_vel_;
            Vector3d &swing_acc = this->left_stance_ ? this->right_foot_acc_ : this->left_foot_acc_;

            // drive the CP to the swing foot's foothold by the end of the step
            this->cp_d_ << this->foothold_x_, swing_pos.y();

            if (this->phi_com_ >= 1.0)
            {
                // land exactly on the foothold, at rest
                this->updateSwingFoot(swing_pos, swing_vel, swing_acc, 1.0);
                swing_vel.setZero();
                swing_acc.setZero();

                // the landed foot becomes the stance foot: ZMP onto it, the other foot swings
                this->left_stance_ = !this->left_stance_;
                this->phi_com_ = 0.0;
                this->zmp_d_ = swing_pos.head<2>();
                    this->startSwing(stance_pos, swing_pos); // old stance foot lifts off
            }
            else
            {
                this->zmp_d_ = this->zmpForTargetCP();
                this->updateSwingFoot(swing_pos, swing_vel, swing_acc, this->phi_com_);
            }
        }

        // integrate the LIP (exact discretization): CP driven by the ZMP, CoM driven by the new CP
        this->ComputeCP(this->zmp_d_);
        this->ComputeCoM(this->cp_);
    }

    // CoM reference at the LIP height
    Vector3d LIPTaskPlanner::getRefCoMPosition() const
    {
        return Vector3d(this->com_.x(), this->com_.y(), this->zc_);
    }

    // CoM velocity from the LIP: d(com)/dt = w (cp - com)
    Vector3d LIPTaskPlanner::getRefCoMVelocity() const
    {
        const Vector2d dcom = this->CoM_dynamics(this->cp_, this->com_);
        return Vector3d(dcom.x(), dcom.y(), 0.0);
    }

    // CoM acceleration from the LIP: d2(com)/dt2 = w (dcp - dcom) = w^2 (com - zmp)
    Vector3d LIPTaskPlanner::getRefCoMAcceleration() const
    {
        const Vector2d ddcom = this->w_ * this->w_ * (this->com_ - this->zmp_d_);
        return Vector3d(ddcom.x(), ddcom.y(), 0.0);
    }

    // DSt in the init phase (both feet on the ground), then the current stance leg
    LegState LIPTaskPlanner::getStanceLeg() const
    {
        if (!this->done_init_walking_) return DSt;
        return this->left_stance_ ? LSt : RSt;
    }

}
