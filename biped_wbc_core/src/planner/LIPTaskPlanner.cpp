#include "biped_wbc/planner/LIPTaskPlanner.hpp"
#include "biped_wbc/common/data_type.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace biped_controllers

{
    // Reads the planner settings from the "LIPPlanner" section of a yaml file, e.g.
    //   LIPPlanner:
    //     Tswing: 1.0               # [s] swing (step) duration
    //     sampling_frequency: 500   # [Hz] planner rate -> dt = 1 / f
    //     com_height: 0.45          # [m] CoM height of the LIP model
    // (the foot keys of the section, e.g. max_step_length, hip_width, are FootPlanner's)
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

        // the yaml gives the rate, the planner works with the sampling time
        double frequency = 1.0 / this->dt_;
        read("sampling_frequency", frequency);
        read("Tswing", this->T_);
        read("com_height", this->zc_);

        // check the values before using them (a zero rate or height would divide by zero later)
        if (this->T_ <= 0.0 || this->zc_ <= 0.0 || frequency <= 0.0)
        {
            throw std::runtime_error("LIPTaskPlanner: Tswing, com_height and sampling_frequency must be > 0 "
                                     "(file '" + CONFIG_YAML_PATH + "')");
        }
        this->dt_ = 1.0 / frequency;

        // LIP natural frequency from the CoM height read above (the member initializer used the
        // default zc_)
        this->w_ = std::sqrt(this->g_ / this->zc_);

        // start like WalkPlanner.py: CoM at the origin
        this->reset(Vector2d::Zero());
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

    // restart from the robot's state: CoM at rest (CP = CoM), init phase next
    void LIPTaskPlanner::reset(const Vector2d &com)
    {
        // LIP state at rest
        this->com_ = com;
        this->cp_ = com;
        this->cp_d_ = com;
        this->zmp_d_ = com;

        // walking state: init phase first
        this->done_init_walking_ = false;
        this->phi_com_ = 0.0;
        this->cp0_ = com;
        this->phase_start_ = true;
    }


    // ZMP, constant over the phase, that brings the CP from cp0_ (its value at the phase start) to
    // cp_d_ at the end of the phase (as walk_planner.cpp of tsid_examples, commit 4de357c).
    // CP solution with a constant ZMP p over T: cp(T) = p + e^{w T} (cp0 - p), solved for p so
    // that cp(T) = cp_d:  p = (cp_d - b cp0) / (1 - b), b = e^{w T}
    Vector2d LIPTaskPlanner::zmpForTargetCP() const
    {
        const double b = std::exp(this->w_ * this->T_);
        return (this->cp_d_ - b * this->cp0_) / (1.0 - b);
    }

    // one planning cycle (WalkPlanner.init_walking / plan_walking, CoM part). The init phase and
    // the steps differ only in cp_target and the ZMP limits (WalkPlanner).
    void LIPTaskPlanner::planWalking(const Vector2d &cp_target, const Vector2d &zmp_min, const Vector2d &zmp_max)
    {
        // timing base: phase of the init phase / current step
        this->phi_com_ += this->dt_ / this->T_;

        // first cycle of a phase: the CP the phase starts from (in the frame of this phase)
        if (this->phase_start_)
        {
            this->cp0_ = this->cp_;
            this->phase_start_ = false;
        }

        // drive the CP to the target by the end of the phase
        this->cp_d_ = cp_target;

        if (this->phi_com_ >= 1.0)
        {
            // end of the phase: the foot at the target becomes the stance foot, ZMP onto it; the
            // next phase is a step
            this->zmp_d_ = this->cp_d_;
            this->phi_com_ = 0.0;
            this->done_init_walking_ = true;
            this->phase_start_ = true;
        }
        else
        {
            // inside the limits (e.g. the stance sole): the CP may then miss cp_d a bit
            this->zmp_d_ = this->zmpForTargetCP().cwiseMax(zmp_min).cwiseMin(zmp_max);
        }

        // integrate the LIP (exact discretization): CP driven by the ZMP, CoM driven by the new CP
        this->ComputeCP(this->zmp_d_);
        this->ComputeCoM(this->cp_);
    }

    void LIPTaskPlanner::setComHeight(double zc)
    {
        if (zc <= 0.0) return;
        this->zc_ = zc;
        this->w_ = std::sqrt(this->g_ / this->zc_);
    }

    // same motion, other frame: every planar point of the state x -> R x + t (the phase is kept)
    void LIPTaskPlanner::transformState(const Eigen::Matrix2d &R, const Vector2d &t)
    {
        this->com_ = R * this->com_ + t;
        this->cp_ = R * this->cp_ + t;
        this->zmp_d_ = R * this->zmp_d_ + t;
        this->cp_d_ = R * this->cp_d_ + t;
        this->cp0_ = R * this->cp0_ + t;
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
}