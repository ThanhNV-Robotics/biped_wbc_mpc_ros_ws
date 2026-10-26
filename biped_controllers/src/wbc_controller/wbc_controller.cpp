#include "wbc_controller/wbc_controller.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>

namespace biped_controllers
{
    // command interfaces the controller writes (called before on_activate, after on_configure):
    // PVT commands of every joint, <joint>/position, <joint>/velocity, <joint>/effort
    controller_interface::InterfaceConfiguration WBCController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto &joint : this->joint_names_)
        {
            config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
            config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
            config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
        }
        return config;
    }

    // state interfaces the controller reads (called before on_activate, after on_configure)
    controller_interface::InterfaceConfiguration WBCController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // IMU: its 10 interfaces <imu_name>/orientation.x ... (names built by the helper)
        if (this->imu_)
        {
            for (const auto &name : this->imu_->get_state_interface_names())
            {
                config.names.push_back(name);
            }
        }
        // joints: measured position, velocity and applied torque of every joint
        for (const auto &joint : this->joint_names_)
        {
            config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
            config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
            config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
        }
        // motors enabled (1) or off (0), e.g. BipedSystem/enabled
        config.names.push_back(this->enabled_interface_);

        return config;
    }

    // ---------------------------------------------------------------------
    // 1st: on_init() (load)
    // Declare the parameters and store their values. No heavy work here: the URDF is loaded and
    // the estimators are built in on_configure().
    // ---------------------------------------------------------------------
    controller_interface::CallbackReturn WBCController::on_init()
    {
        // logger of this controller's node (messages appear as [wbc_controller])
        const auto logger = get_node()->get_logger();

        // defaults of the URDF location; a value in controllers.yaml overrides them
        const std::string default_description_pkg = "biped_description";
        const std::string default_urdf = "urdf/v2_biped_robot_12dof.urdf";
        const std::string default_wbc_pkg = "biped_bringup";
        const std::string default_wbc_yaml = "config/tsid_wbc.yaml";

        try
        {
            // auto_declare(name, default) registers the parameter and returns the value in effect:
            // the yaml value if set, otherwise the default

            // URDF: package name + file path inside the package's share directory
            this->urdf_package_ = auto_declare<std::string>("urdf_package", default_description_pkg);
            this->urdf_file_ = auto_declare<std::string>("urdf_file", default_urdf);
            // WBC yaml (TSID tasks, contacts, LIP planner): package + file inside its share directory
            this->wbc_config_package_ = auto_declare<std::string>("wbc_config_package", default_wbc_pkg);
            this->wbc_controller_config_path = auto_declare<std::string>("wbc_tsid_yaml", default_wbc_yaml);
            // the 12 controlled joints; no default, the list must come from controllers.yaml
            // (an empty list is rejected in on_configure)
            this->joint_names_ = auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
            // URDF frames of the feet, used by the state estimator and the contact fusion
            this->left_foot_frame_ = auto_declare<std::string>("left_foot_frame", "left_ankle_pitch_link");
            this->right_foot_frame_ = auto_declare<std::string>("right_foot_frame", "right_ankle_pitch_link");
            // ros2_control sensor name of the IMU: its interfaces are <imu_name>/orientation.x, ...
            this->imu_name_ = auto_declare<std::string>("imu_name", "imu");
            // hardware state interface telling whether the motors are enabled (1) or off (0)
            this->enabled_interface_ = auto_declare<std::string>("enabled_interface", "BipedSystem/enabled");
        }
        catch (const std::exception &e) // e.g. a yaml value of the wrong type
        {
            // fail the load cleanly instead of letting the exception reach the controller manager
            RCLCPP_ERROR(logger, "on_init: declaring the parameters failed: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        // print the values in effect, to see whether the yaml or the defaults were used
        RCLCPP_INFO(logger, "URDF: package '%s', file '%s'%s",
                    this->urdf_package_.c_str(), this->urdf_file_.c_str(),
                    (this->urdf_package_ == default_description_pkg && this->urdf_file_ == default_urdf) ? " (defaults)" : "");
        RCLCPP_INFO(logger, "%zu joints, feet '%s' / '%s', IMU '%s', enabled interface '%s'",
                    this->joint_names_.size(), this->left_foot_frame_.c_str(), this->right_foot_frame_.c_str(),
                    this->imu_name_.c_str(), this->enabled_interface_.c_str());

        // SUCCESS: the controller is loaded (state "unconfigured"); next is on_configure()
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WBCController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        // re-read the parameters: they may have been changed (ros2 param set) after on_init, while
        // the controller was unconfigured; this is the value in effect from now on
        this->urdf_package_ = get_node()->get_parameter("urdf_package").as_string();
        this->urdf_file_ = get_node()->get_parameter("urdf_file").as_string();
        this->wbc_config_package_ = get_node()->get_parameter("wbc_config_package").as_string();
        this->wbc_controller_config_path = get_node()->get_parameter("wbc_tsid_yaml").as_string();
        this->joint_names_ = get_node()->get_parameter("joints").as_string_array();
        this->left_foot_frame_ = get_node()->get_parameter("left_foot_frame").as_string();
        this->right_foot_frame_ = get_node()->get_parameter("right_foot_frame").as_string();
        this->imu_name_ = get_node()->get_parameter("imu_name").as_string();
        this->enabled_interface_ = get_node()->get_parameter("enabled_interface").as_string();

        // build the control modules (URDF, yaml); fail the configure step if that is not possible
        if (!this->InitControlModules())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // check the joint list against the model and build the joint order maps (needs the model
        // of QP_TSID, so after InitControlModules)
        if (!this->BuildJointMap())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // IMU helper: the names of its 10 state interfaces come from imu_name_ (= <sensor name> in
        // biped_hw.urdf.xacro); it is claimed in state_interface_configuration()
        this->imu_ = std::make_unique<semantic_components::IMUSensor>(this->imu_name_);

        // per-cycle buffers with their final sizes: update() never allocates
        this->AllocateBuffers();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WBCController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        const auto logger = get_node()->get_logger();

        // IMU: bind the helper to its 10 loaned state interfaces (found by name); fails if the
        // hardware has no sensor <imu_name> (e.g. imu_name differs from the xacro)
        if (!this->imu_->assign_loaned_state_interfaces(this->state_interfaces_))
        {
            RCLCPP_ERROR(logger, "on_activate: IMU '%s' state interfaces not found", this->imu_name_.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        // first reading, to check the IMU once (orientation is {x, y, z, w}; standing still the
        // acceleration is about (0, 0, +9.81))
        const auto q = this->imu_->get_orientation();
        const auto a = this->imu_->get_linear_acceleration();
        RCLCPP_INFO(logger, "IMU '%s': orientation (x y z w) %.3f %.3f %.3f %.3f, acceleration %.2f %.2f %.2f",
                    this->imu_name_.c_str(), q[0], q[1], q[2], q[3], a[0], a[1], a[2]);

        // joints: find each joint's loaned interfaces by name (the loan order is not guaranteed);
        // find() returns loans.size() if the interface is missing
        const auto find = [](const auto &loans, const std::string &joint, const char *interface) {
            for (std::size_t k = 0; k < loans.size(); ++k)
            {
                if (loans[k].get_prefix_name() == joint && loans[k].get_interface_name() == interface)
                {
                    return k;
                }
            }
            return loans.size();
        };
        const std::size_t n = this->joint_names_.size();
        this->pos_cmd_idx_.assign(n, 0);
        this->vel_cmd_idx_.assign(n, 0);
        this->eff_cmd_idx_.assign(n, 0);
        this->pos_state_idx_.assign(n, 0);
        this->vel_state_idx_.assign(n, 0);
        this->eff_state_idx_.assign(n, 0);
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::string &joint = this->joint_names_[i];
            this->pos_cmd_idx_[i] = find(this->command_interfaces_, joint, hardware_interface::HW_IF_POSITION);
            this->vel_cmd_idx_[i] = find(this->command_interfaces_, joint, hardware_interface::HW_IF_VELOCITY);
            this->eff_cmd_idx_[i] = find(this->command_interfaces_, joint, hardware_interface::HW_IF_EFFORT);
            this->pos_state_idx_[i] = find(this->state_interfaces_, joint, hardware_interface::HW_IF_POSITION);
            this->vel_state_idx_[i] = find(this->state_interfaces_, joint, hardware_interface::HW_IF_VELOCITY);
            this->eff_state_idx_[i] = find(this->state_interfaces_, joint, hardware_interface::HW_IF_EFFORT);

            const std::size_t nc = this->command_interfaces_.size(), ns = this->state_interfaces_.size();
            if (this->pos_cmd_idx_[i] == nc || this->vel_cmd_idx_[i] == nc || this->eff_cmd_idx_[i] == nc ||
                this->pos_state_idx_[i] == ns || this->vel_state_idx_[i] == ns || this->eff_state_idx_[i] == ns)
            {
                RCLCPP_ERROR(logger, "on_activate: missing interfaces for joint '%s'", joint.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
        }

        // motors: find the enabled interface and refuse to activate if the motors are off (the WBC
        // would command torques the drives do not apply)
        this->enabled_state_idx_ = this->state_interfaces_.size();
        for (std::size_t k = 0; k < this->state_interfaces_.size(); ++k)
        {
            if (this->state_interfaces_[k].get_name() == this->enabled_interface_)
            {
                this->enabled_state_idx_ = k;
            }
        }
        if (this->enabled_state_idx_ == this->state_interfaces_.size())
        {
            RCLCPP_ERROR(logger, "on_activate: missing state interface '%s'", this->enabled_interface_.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        if (this->state_interfaces_[this->enabled_state_idx_].get_value() < 0.5)
        {
            RCLCPP_ERROR(logger, "on_activate: motors are disabled ('%s' = 0); enable the hardware first",
                         this->enabled_interface_.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // hold command, bumpless: keep the position command the drives already hold (the hardware's
        // own hold, or the previous controller's last command). Not the measured q: under gravity the
        // PD hold lags its command (ankle ~0.007 rad), and that lag is the torque holding the robot
        // up; commanding p = q would drop it and the robot would sag. Measured q only if the
        // command is not set (NaN). Zero velocity and torque; written now, so the drives get it
        // before the first update()
        for (std::size_t i = 0; i < n; ++i)
        {
            const double q = this->state_interfaces_[this->pos_state_idx_[i]].get_value();
            if (!std::isfinite(q))
            {
                RCLCPP_ERROR(logger, "on_activate: joint '%s' has no valid position", this->joint_names_[i].c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            const double p = this->command_interfaces_[this->pos_cmd_idx_[i]].get_value();
            this->q_cmd_(i) = std::isfinite(p) ? p : q;
            this->v_cmd_(i) = 0.0;
            this->tau_cmd_(i) = 0.0;
            this->command_interfaces_[this->pos_cmd_idx_[i]].set_value(this->q_cmd_(i));
            this->command_interfaces_[this->vel_cmd_idx_[i]].set_value(this->v_cmd_(i));
            this->command_interfaces_[this->eff_cmd_idx_[i]].set_value(this->tau_cmd_(i));
        }

        // contact probability: the robot starts standing on both feet (same as the initial state of
        // the contact fusion); used by the state estimator in the first update()
        this->contact_probability_.setOnes();

        RCLCPP_INFO(logger, "Active: %zu joints claimed (position / velocity / effort), holding the current pose", n);

        // TODO: reset estimators / planner (B3)

        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn WBCController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        // IMU: drop the references to the loaned interfaces before the controller manager takes
        // them back
        this->imu_->release_interfaces();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type WBCController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        const auto logger = get_node()->get_logger();

        // Get joystick command

        // motors disabled (hardware fault / command timeout): no control, hold command re-synced to
        // the measured position so there is no jump once the motors are enabled again
        if (this->state_interfaces_[this->enabled_state_idx_].get_value() < 0.5)
        {
            RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000,
                                  "motors disabled ('%s' = 0): holding the measured pose", this->enabled_interface_.c_str());
            for (std::size_t i = 0; i < this->joint_names_.size(); ++i)
            {
                const double q = this->state_interfaces_[this->pos_state_idx_[i]].get_value();
                if (std::isfinite(q))
                    this->q_cmd_(i) = q;
                this->v_cmd_(i) = 0.0;
                this->tau_cmd_(i) = 0.0;
            }
            this->WriteCommands();
            return controller_interface::return_type::OK;
        }

        // Read robot feedback measurement (IMU + joint states); the hardware read() was already
        // called by the controller manager this cycle. Invalid reading -> keep the last command
        if (!this->ReadSensors())
        {
            RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000,
                                  "invalid sensor reading (NaN / Inf): keeping the last command");
            this->WriteCommands();
            return controller_interface::return_type::OK;
        }

        // B2 check (temporary): one joint and the IMU once per second
        RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), 1000,
                             "q[0] %.3f dq[0] %.3f | quat (w x y z) %.3f %.3f %.3f %.3f | acc %.2f %.2f %.2f",
                             this->sensor_.actuator_state.qj(0), this->sensor_.actuator_state.dqj(0),
                             this->sensor_.imu_sensor.imu_quat_.w(), this->sensor_.imu_sensor.imu_quat_.x(),
                             this->sensor_.imu_sensor.imu_quat_.y(), this->sensor_.imu_sensor.imu_quat_.z(),
                             this->sensor_.imu_sensor.imu_accel_L(0), this->sensor_.imu_sensor.imu_accel_L(1),
                             this->sensor_.imu_sensor.imu_accel_L(2));

        // estimate state from hardware interface

        // estimate contact state

        // compute tsid_wbc torque command
            // set task based on joystick command

            // Process the task
                // command 1: homing, postural task only!, control all the joints to home position 

                // command 2: init walking (postural task only!), lower down CoM to reference zc (bend the knee )

                // command 3: walking with const step length and swing time

                // command 4: heading turning

                // command 5: transition from walking to stop

        // send command hardware using command interface (B2: still the hold command of on_activate)
        this->WriteCommands();

        return controller_interface::return_type::OK;
    }

    // ---------------------------------------------------------------------
    // Joint + IMU state interfaces -> sensor_ (joints in Pinocchio order). Real-time: no
    // allocation, only writes into the buffers of AllocateBuffers().
    // Returns false if a value is not finite.
    // ---------------------------------------------------------------------
    bool WBCController::ReadSensors()
    {
        // joints: ros2_control index i -> Pinocchio index j
        auto &joints = this->sensor_.actuator_state;
        for (std::size_t i = 0; i < this->joint_names_.size(); ++i)
        {
            const int j = this->ros_to_pin_[i];
            joints.qj(j) = this->state_interfaces_[this->pos_state_idx_[i]].get_value();
            joints.dqj(j) = this->state_interfaces_[this->vel_state_idx_[i]].get_value();
            joints.torquej(j) = this->state_interfaces_[this->eff_state_idx_[i]].get_value();
        }

        // IMU: orientation comes as {x, y, z, w}, Eigen's quaternion constructor takes (w, x, y, z);
        // gyro and accelerometer are in the IMU frame (copied to locals, std::array by value)
        auto &imu = this->sensor_.imu_sensor;
        const auto o = this->imu_->get_orientation();
        const auto w = this->imu_->get_angular_velocity();
        const auto a = this->imu_->get_linear_acceleration();
        imu.imu_quat_ = Quat(o[3], o[0], o[1], o[2]);
        imu.imu_gyro_L << w[0], w[1], w[2];
        imu.imu_accel_L << a[0], a[1], a[2];

        // all values finite (a broken interface reads NaN)
        return joints.qj.allFinite() && joints.dqj.allFinite() && joints.torquej.allFinite() &&
               imu.imu_quat_.coeffs().allFinite() && imu.imu_gyro_L.allFinite() && imu.imu_accel_L.allFinite();
    }

    // ---------------------------------------------------------------------
    // q_cmd_ / v_cmd_ / tau_cmd_ (ros2_control order) -> command interfaces. Called every cycle,
    // also when nothing changes: the hardware's command timeout needs a fresh command.
    // ---------------------------------------------------------------------
    void WBCController::WriteCommands()
    {
        for (std::size_t i = 0; i < this->joint_names_.size(); ++i)
        {
            this->command_interfaces_[this->pos_cmd_idx_[i]].set_value(this->q_cmd_(i));
            this->command_interfaces_[this->vel_cmd_idx_[i]].set_value(this->v_cmd_(i));
            this->command_interfaces_[this->eff_cmd_idx_[i]].set_value(this->tau_cmd_(i));
        }
    }


    // ---------------------------------------------------------------------
    // Creates the control modules (called from on_configure). Order matters: QP_TSID owns the
    // robot model, the state estimator and the contact fusion reference it.
    // Returns false (error logged) if a file is missing or invalid.
    // ---------------------------------------------------------------------
    bool WBCController::InitControlModules ()
    {
        const auto logger = get_node()->get_logger();

        // 1. file paths: package share directory + relative path (parameters from on_init)
        std::string urdf_path, wbc_yaml_path;
        try
        {
            urdf_path = ament_index_cpp::get_package_share_directory(this->urdf_package_) + "/" + this->urdf_file_;
            wbc_yaml_path = ament_index_cpp::get_package_share_directory(this->wbc_config_package_) + "/" +
                            this->wbc_controller_config_path;
        }
        catch (const std::exception &e) // package not found
        {
            RCLCPP_ERROR(logger, "InitControlModules: package not found: %s", e.what());
            return false;
        }
        RCLCPP_INFO(logger, "URDF: %s", urdf_path.c_str());
        RCLCPP_INFO(logger, "WBC config: %s", wbc_yaml_path.c_str());

        try
        {
            // 2. QP_TSID: loads the URDF (free-flyer robot), the TSID formulation and the contacts
            this->qp_tsid_controller_ = std::make_unique<QP_TSID>(wbc_yaml_path, urdf_path);
            tsid::robots::RobotWrapper &robot = this->qp_tsid_controller_->getRobotWrapper();
            RCLCPP_INFO(logger, "Robot model: nq %d, nv %d, na %d", robot.nq(), robot.nv(), robot.na());

            // 3. LIP task planner: section "LIPPlanner" of the same yaml
            this->lip_task_planner_ = std::make_unique<LIPTaskPlanner>(wbc_yaml_path);

            // control period of all modules: sampling_frequency of the yaml. get_update_rate() is 0
            // unless the controller has its own update_rate parameter (0 = every controller_manager
            // cycle, whose rate the controller cannot read), so it is only used as a cross-check.
            // sampling_frequency must equal controller_manager.update_rate (controllers.yaml).
            const double dt = this->lip_task_planner_->getDt();
            if (get_update_rate() > 0 && std::abs(dt * get_update_rate() - 1.0) > 1e-6)
            {
                RCLCPP_WARN(logger, "LIPPlanner sampling_frequency (%.0f Hz) differs from the controller update rate "
                            "(%u Hz): the plan and the estimators run at the wrong speed",
                            1.0 / dt, get_update_rate());
            }

            // 4. state estimator: base position / velocity from IMU + leg kinematics
            this->state_estimator_ = std::make_unique<StateEstimator>(robot, dt, this->left_foot_frame_,
                                                                      this->right_foot_frame_);

            // 5. contact fusion: foot contact probability from the momentum observer, the foot height
            //    and the gait phase (observer cutoff 15 Hz)
            this->contact_fusion_ = std::make_unique<ContactFusion>(
                robot.model(), std::vector<std::string>{this->left_foot_frame_, this->right_foot_frame_}, dt, 15.0);

            // sole corners of each foot: bottom face of the foot's collision box in the URDF (the
            // same source the TSID contacts can use), for the foot height of the contact fusion
            const std::string feet[2] = {this->left_foot_frame_, this->right_foot_frame_};
            for (std::size_t i = 0; i < 2; ++i)
            {
                Eigen::Matrix3Xd corners;
                if (this->qp_tsid_controller_->soleCornersFromUrdf(feet[i], Eigen::Vector3d::UnitZ(), corners))
                {
                    this->contact_fusion_->setSoleCorners(i, corners);
                }
                else
                {
                    RCLCPP_WARN(logger, "No sole box in the URDF for foot frame '%s': the contact fusion uses the "
                                "frame origin as the sole", feet[i].c_str());
                }
            }
        }
        catch (const std::exception &e) // missing / invalid file or frame (the constructors throw)
        {
            RCLCPP_ERROR(logger, "InitControlModules: %s", e.what());
            return false;
        }

        RCLCPP_INFO(logger, "Control modules ready: QP_TSID, LIPTaskPlanner, StateEstimator, ContactFusion (dt %.4f s)",
                    this->lip_task_planner_->getDt());
        return true;
    }

    // ---------------------------------------------------------------------
    // Checks the `joints` parameter against the robot model and builds the index maps
    // ros2_control order <-> Pinocchio order (called from on_configure, after InitControlModules).
    // The interfaces are claimed in the order of joint_names_; TSID and the estimators work in
    // the order of the model's actuated joints (q[7:], v[6:]). Every conversion between the two
    // uses these maps, so the order in controllers.yaml is free.
    // ---------------------------------------------------------------------
    bool WBCController::BuildJointMap ()
    {
        const auto logger = get_node()->get_logger();
        const pinocchio::Model &model = this->qp_tsid_controller_->getRobotWrapper().model();
        const int na = this->qp_tsid_controller_->getRobotWrapper().na(); // actuated joints (12)

        // one name per actuated joint of the model
        if (static_cast<int>(this->joint_names_.size()) != na)
        {
            RCLCPP_ERROR(logger, "BuildJointMap: 'joints' has %zu names, the model has %d actuated joints "
                         "(set 'joints' in controllers.yaml)", this->joint_names_.size(), na);
            return false;
        }

        this->ros_to_pin_.assign(na, -1);
        this->pin_to_ros_.assign(na, -1);
        for (int i = 0; i < na; ++i)
        {
            const std::string &name = this->joint_names_[i];

            // the joint must exist in the URDF model (getJointId returns model.njoints if not)
            const pinocchio::JointIndex jid = model.getJointId(name);
            if (jid >= static_cast<pinocchio::JointIndex>(model.njoints))
            {
                RCLCPP_ERROR(logger, "BuildJointMap: joint '%s' is not in the robot model", name.c_str());
                return false;
            }

            // a 1-DoF joint after the free flyer: its place among the actuated joints is its
            // velocity index minus the 6 base velocities (joint 0 = universe, 1 = free flyer)
            if (model.joints[jid].nv() != 1 || model.joints[jid].idx_v() < 6)
            {
                RCLCPP_ERROR(logger, "BuildJointMap: joint '%s' is not an actuated 1-DoF joint", name.c_str());
                return false;
            }
            const int j = model.joints[jid].idx_v() - 6;

            // each joint only once
            if (this->pin_to_ros_[j] != -1)
            {
                RCLCPP_ERROR(logger, "BuildJointMap: joint '%s' is listed twice in 'joints'", name.c_str());
                return false;
            }
            this->ros_to_pin_[i] = j;
            this->pin_to_ros_[j] = i;
        }

        // log the mapping once, to check the order against the URDF
        std::string mapping;
        for (int i = 0; i < na; ++i)
        {
            mapping += "\n  " + this->joint_names_[i] + " -> " + std::to_string(this->ros_to_pin_[i]);
        }
        RCLCPP_INFO(logger, "Joint map (ros2_control -> Pinocchio):%s", mapping.c_str());
        return true;
    }

    // ---------------------------------------------------------------------
    // Sizes the per-cycle buffers once (called from on_configure, after InitControlModules and
    // BuildJointMap). update() only writes into them: an Eigen::VectorXd that is created or
    // resized allocates heap memory, which must not happen in the real-time loop.
    // ---------------------------------------------------------------------
    void WBCController::AllocateBuffers ()
    {
        const tsid::robots::RobotWrapper &robot = this->qp_tsid_controller_->getRobotWrapper();
        const int nq = robot.nq(); // 19: base position 3 + quaternion 4 + 12 joints
        const int nv = robot.nv(); // 18: base twist 6 + 12 joints
        const int na = robot.na(); // 12 actuated joints

        // sensor data of one cycle (joint vectors of size na inside, IMU fixed-size)
        this->sensor_ = RobotSensor(na);

        // estimated state in Pinocchio / TSID convention
        this->q_ = Eigen::VectorXd::Zero(nq);
        this->v_ = Eigen::VectorXd::Zero(nv);

        // contact probability per foot [left, right]; reset to [1, 1] in on_activate
        this->contact_probability_ = Eigen::VectorXd::Ones(2);

        // TSID solution
        this->dv_ = Eigen::VectorXd::Zero(nv);
        this->tau_ = Eigen::VectorXd::Zero(na);

        // commands for the 12 joints, in ros2_control order (written to the command interfaces)
        this->q_cmd_ = Eigen::VectorXd::Zero(na);
        this->v_cmd_ = Eigen::VectorXd::Zero(na);
        this->tau_cmd_ = Eigen::VectorXd::Zero(na);
    }

} // namespace biped_controllers

PLUGINLIB_EXPORT_CLASS(biped_controllers::WBCController, controller_interface::ControllerInterface)
