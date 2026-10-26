#include "biped_wbc/QP_tsid/QPtsid.hpp"
#include <urdf_parser/urdf_parser.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <urdf_parser/urdf_parser.h>

namespace biped_controllers
{
    // constructor
    QP_TSID::QP_TSID(const std::string &yaml_config_path, const std::string &urdf_path)
    {
        this->yaml_config_path_ = yaml_config_path;
        this->urdf_path_ = urdf_path;

        this->yaml_config_ = YAML::LoadFile(yaml_config_path); // read yaml config file

        std::vector<std::string> package_dirs;
        this->robot_wrapper_ = std::make_shared<tsid::robots::RobotWrapper>(urdf_path,
                                                                            package_dirs, pinocchio::JointModelFreeFlyer(), false);

        // TSID formulation on this robot (created after it: it needs the robot in its constructor)
        this->tsid_ = std::make_unique<tsid::InverseDynamicsFormulationAccForce>("tsid", *this->robot_wrapper_, false);

        // create the tasks from the yaml (not added to the TSID formulation yet)
        this->setupContactTask(); // first: it sets the foot frames used by the swing tasks
        this->setupSwingTask();
        this->setupCoMTask();
        this->setupBaseOrientationTask();
        this->setupPostureTask();
        this->setupActuationBoundsTask();
    }

    bool QP_TSID::soleCornersFromUrdf(const std::string &link_name, const Eigen::Vector3d &normal,
                                      Eigen::Matrix3Xd &corners) const
    {
        if (this->urdf_path_.empty())
            return false;
        const urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(this->urdf_path_);
        if (!urdf_model)
        {
            std::cerr << "[tsidTaskParser] Warning: failed to parse URDF '" << this->urdf_path_ << "'" << std::endl;
            return false;
        }
        const urdf::LinkConstSharedPtr link = urdf_model->getLink(link_name);
        if (!link)
        {
            std::cerr << "[tsidTaskParser] Warning: '" << link_name
                      << "' is not a URDF link, cannot read its collision geometry" << std::endl;
            return false;
        }

        // Among the link's collision boxes, take the face pointing most against the
        // contact normal (the sole); with several boxes, the one lowest along -normal.
        // URDF link frames coincide with Pinocchio's body frames, so the result is
        // directly in the contact frame.
        double best_height = std::numeric_limits<double>::infinity();
        for (const auto &collision : link->collision_array)
        {
            if (!collision || !collision->geometry || collision->geometry->type != urdf::Geometry::BOX)
                continue;
            const auto box = std::static_pointer_cast<const urdf::Box>(collision->geometry);
            const urdf::Pose &pose = collision->origin;
            const Eigen::Vector3d center(pose.position.x, pose.position.y, pose.position.z);
            const Eigen::Matrix3d R =
                Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z).toRotationMatrix();
            const Eigen::Vector3d half(box->dim.x / 2.0, box->dim.y / 2.0, box->dim.z / 2.0);

            int k = 0;
            for (int a = 1; a < 3; ++a)
            {
                if (std::abs(R.col(a).dot(normal)) > std::abs(R.col(k).dot(normal)))
                    k = a;
            }
            const double s = (R.col(k).dot(normal) > 0.0) ? -1.0 : 1.0;
            const Eigen::Vector3d face_center = center + s * half(k) * R.col(k);
            const double height = face_center.dot(normal);
            if (height >= best_height)
                continue;
            best_height = height;

            // Corners ordered (-i,-j), (-i,+j), (+i,-j), (+i,+j) over the face's two in-plane axes
            const int i = (k == 0) ? 1 : 0;
            const int j = 3 - k - i;
            corners.resize(3, 4);
            int c = 0;
            for (double si : {-1.0, 1.0})
            {
                for (double sj : {-1.0, 1.0})
                {
                    corners.col(c++) = face_center + si * half(i) * R.col(i) + sj * half(j) * R.col(j);
                }
            }
        }
        if (!std::isfinite(best_height))
        {
            std::cerr << "[tsidTaskParser] Warning: link '" << link_name << "' has no collision box" << std::endl;
            return false;
        }
        return true;
    }

    YAML::Node QP_TSID::contactConfig(const std::string &side) const
    {
        // the contact_tasks entry of this foot: its contact_frame contains "left" / "right"
        for (const auto &entry : this->yaml_config_["contact_tasks"])
        {
            if (entry["contact_frame"].as<std::string>().find(side) != std::string::npos)
                return entry;
        }
        throw std::runtime_error("[QP_TSID] no contact_tasks entry for the " + side + " foot in '" +
                                 this->yaml_config_path_ + "'");
    }

    void QP_TSID::setupContactTask()
    {
        // one Contact6d per foot (left_foot_contact_task, right_foot_contact_task), same settings
        // layout: a lambda builds one from its yaml entry
        auto make_contact = [this](const std::string &side, std::string &frame, double &force_reg_weight)
        {
            const YAML::Node cfg = this->contactConfig(side);

            // contact frame: must exist in the model
            frame = cfg["contact_frame"].as<std::string>();
            if (!this->robot_wrapper_->model().existFrame(frame))
                throw std::runtime_error("[QP_TSID] contact frame '" + frame + "' is not in the URDF");

            // sole corners (in the contact frame) from the foot's collision box in the URDF;
            // Contact6d needs exactly 4 points
            const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
            Eigen::Matrix3Xd corners;
            if (!this->soleCornersFromUrdf(frame, normal, corners) || corners.cols() != 4)
                throw std::runtime_error("[QP_TSID] no 4-corner sole for '" + frame + "' in the URDF");

            // friction cone and normal force bounds
            const double mu = cfg["mu"].as<double>();
            const double f_min = cfg["f_min"].as<double>();
            const double f_max = cfg["f_max"].as<double>();
            auto contact = std::make_shared<tsid::contacts::Contact6d>(
                "contact_" + side, *this->robot_wrapper_, frame, corners, normal, mu, f_min, f_max);

            // contact acceleration gains (kd defaults to 2*sqrt(kp), critical damping)
            const double kp = cfg["kp"].as<double>();
            const double kd = cfg["kd"] ? cfg["kd"].as<double>() : 2.0 * std::sqrt(kp);
            contact->Kp(kp * Eigen::VectorXd::Ones(6));
            contact->Kd(kd * Eigen::VectorXd::Ones(6));

            // force regularization weight, used by addRigidContact
            force_reg_weight = cfg["weight"] ? cfg["weight"].as<double>() : 1e-3;
            return contact;
        };

        this->left_foot_contact_task = make_contact("left", this->left_foot_frame_, this->left_contact_force_reg_weight_);
        this->right_foot_contact_task = make_contact("right", this->right_foot_frame_, this->right_contact_force_reg_weight_);
    }

    void QP_TSID::setupSwingTask()
    {
        // one SE3 task per foot (same frames as the contacts, so setupContactTask runs first),
        // same gains for both, from the swing_foot_task section
        const YAML::Node cfg = this->yaml_config_["swing_foot_task"];
        const double kp = cfg["kp"].as<double>();
        const double kd = cfg["kd"] ? cfg["kd"].as<double>() : 2.0 * std::sqrt(kp);
        this->swing_setting_.weight = cfg["weight"].as<double>();
        this->swing_setting_.priority = cfg["priority"] ? cfg["priority"].as<unsigned int>() : 1;
        this->contact_transition_time_ = cfg["contact_transition_time"] ? cfg["contact_transition_time"].as<double>() : 0.15;

        auto make_swing = [this, kp, kd](const std::string &side, const std::string &frame)
        {
            auto task = std::make_shared<tsid::tasks::TaskSE3Equality>("swing_" + side, *this->robot_wrapper_, frame);
            task->Kp(kp * Eigen::VectorXd::Ones(6));
            task->Kd(kd * Eigen::VectorXd::Ones(6));
            // the planner gives the foot trajectory in the world frame
            task->useLocalFrame(false);
            return task;
        };

        this->left_foot_swing_task = make_swing("left", this->left_foot_frame_);
        this->right_foot_swing_task = make_swing("right", this->right_foot_frame_);
    }

    void QP_TSID::setupCoMTask()
    {
        const YAML::Node cfg = this->yaml_config_["com_task"];

        // gains (kd defaults to 2*sqrt(kp))
        const double kp = cfg["kp"].as<double>();
        const double kd = cfg["kd"] ? cfg["kd"].as<double>() : 2.0 * std::sqrt(kp);
        this->com_task_ = std::make_shared<tsid::tasks::TaskComEquality>("com", *this->robot_wrapper_);
        this->com_task_->Kp(kp * Eigen::Vector3d::Ones());
        this->com_task_->Kd(kd * Eigen::Vector3d::Ones());

        // controlled axes [x, y, z]
        if (cfg["mask"])
        {
            const auto mask = cfg["mask"].as<std::vector<double>>();
            if (mask.size() != 3)
                throw std::runtime_error("[QP_TSID] com_task/mask must have 3 values");
            this->com_task_->setMask(Eigen::Map<const Eigen::Vector3d>(mask.data()));
        }

        this->com_setting_.weight = cfg["weight"].as<double>();
        this->com_setting_.priority = cfg["priority"] ? cfg["priority"].as<unsigned int>() : 1;
    }

    void QP_TSID::setupBaseOrientationTask()
    {
        const YAML::Node cfg = this->yaml_config_["base_orientation_task"];

        // base frame: must exist in the model
        const std::string frame = cfg["frame"].as<std::string>();
        if (!this->robot_wrapper_->model().existFrame(frame))
            throw std::runtime_error("[QP_TSID] base frame '" + frame + "' is not in the URDF");

        // gains (kd defaults to 2*sqrt(kp))
        const double kp = cfg["kp"].as<double>();
        const double kd = cfg["kd"] ? cfg["kd"].as<double>() : 2.0 * std::sqrt(kp);
        this->base_orientation_task_ = std::make_shared<tsid::tasks::TaskSE3Equality>("base_orientation", *this->robot_wrapper_, frame);
        this->base_orientation_task_->Kp(kp * Eigen::VectorXd::Ones(6));
        this->base_orientation_task_->Kd(kd * Eigen::VectorXd::Ones(6));

        // controlled axes [x, y, z, roll, pitch, yaw]: by default orientation only
        Eigen::VectorXd mask(6);
        mask << 0, 0, 0, 1, 1, 1; // just control roll, pitch, yaw
        if (cfg["mask"])
        {
            const auto m = cfg["mask"].as<std::vector<double>>();
            if (m.size() != 6)
                throw std::runtime_error("[QP_TSID] base_orientation_task/mask must have 6 values");
            mask = Eigen::Map<const Eigen::VectorXd>(m.data(), 6);
        }
        this->base_orientation_task_->setMask(mask);

        this->base_orientation_setting_.weight = cfg["weight"].as<double>();
        this->base_orientation_setting_.priority = cfg["priority"] ? cfg["priority"].as<unsigned int>() : 1;
    }

    void QP_TSID::setupPostureTask()
    {
        const YAML::Node cfg = this->yaml_config_["posture_task"];
        const int na = this->robot_wrapper_->na();

        // gains on every actuated joint (kd defaults to 2*sqrt(kp))
        const double kp = cfg["kp"] ? cfg["kp"].as<double>() : 10.0;
        const double kd = cfg["kd"] ? cfg["kd"].as<double>() : 2.0 * std::sqrt(kp);
        this->posture_task_ = std::make_shared<tsid::tasks::TaskJointPosture>("posture", *this->robot_wrapper_);
        this->posture_task_->Kp(kp * Eigen::VectorXd::Ones(na));
        this->posture_task_->Kd(kd * Eigen::VectorXd::Ones(na));

        this->posture_setting_.weight = cfg["weight"].as<double>();
        this->posture_setting_.priority = cfg["priority"] ? cfg["priority"].as<unsigned int>() : 1;
    }

    void QP_TSID::setupActuationBoundsTask()
    {
        const YAML::Node cfg = this->yaml_config_["actuation_bounds_task"];
        const int na = this->robot_wrapper_->na();

        // torque limits -tau_max <= tau <= tau_max, in the model's joint order (Pinocchio order).
        // Default: the URDF <limit effort="..."> of each joint (effortLimit is nv long, the
        // actuated joints are its last na entries)
        Eigen::VectorXd upper = this->robot_wrapper_->model().effortLimit.tail(na);

        // optional override from the yaml (e.g. while the URDF still has placeholder limits)
        if (cfg["tau_max"])
        {
            const auto tau_max = cfg["tau_max"].as<std::vector<double>>();
            if (static_cast<int>(tau_max.size()) != na)
                throw std::runtime_error("[QP_TSID] actuation_bounds_task/tau_max has " + std::to_string(tau_max.size()) +
                                         " values, the robot has " + std::to_string(na) + " joints");
            upper = Eigen::Map<const Eigen::VectorXd>(tau_max.data(), na);
        }
        this->actuation_bounds_task_ = std::make_shared<tsid::tasks::TaskActuationBounds>("actuation_bounds", *this->robot_wrapper_);
        this->actuation_bounds_task_->setBounds(-upper, upper);

        this->actuation_bounds_setting_.weight = cfg["weight"] ? cfg["weight"].as<double>() : 1.0;
        this->actuation_bounds_setting_.priority = cfg["priority"] ? cfg["priority"].as<unsigned int>() : 0;
    }
} // namespace biped_controllers