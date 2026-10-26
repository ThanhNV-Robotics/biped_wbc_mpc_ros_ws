#include "wbc_controller/wbc_controller.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>

namespace biped_controllers
{
    // interfaces the controller claims (called after on_configure, before on_activate): the names
    // come from RobotInterface (joint PVT commands; joint states, IMU, motors enabled)
    controller_interface::InterfaceConfiguration WBCController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        if (this->robot_)
        {
            config.names = this->robot_->commandInterfaceNames();
        }
        return config;
    }

    controller_interface::InterfaceConfiguration WBCController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        if (this->robot_)
        {
            config.names = this->robot_->stateInterfaceNames();
        }
        return config;
    }

    // ---------------------------------------------------------------------
    // 1st: on_init() (load)
    // Declare the parameters (values read in on_configure). No heavy work here: the URDF is loaded
    // and the estimators are built in on_configure().
    // ---------------------------------------------------------------------
    controller_interface::CallbackReturn WBCController::on_init()
    {
        // WBC yaml (TSID tasks, contacts, LIP planner): the only default specific to this controller,
        // the others are the ControllerParams member defaults
        this->params_.config_file = "config/tsid_wbc.yaml";

        try
        {
            // declare every parameter; a value in controllers.yaml overrides the default
            this->params_.declare(*get_node());
        }
        catch (const std::exception &e) // e.g. a yaml value of the wrong type
        {
            // fail the load cleanly instead of letting the exception reach the controller manager
            RCLCPP_ERROR(get_node()->get_logger(), "on_init: declaring the parameters failed: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        // SUCCESS: the controller is loaded (state "unconfigured"); next is on_configure()
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WBCController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // parameters in effect (they may have been changed with ros2 param set after on_init) and
        // the resolved URDF / config paths
        if (!this->params_.read(*get_node()))
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // control modules: QP_TSID (robot model) + planner, then the estimators on that model
        if (!this->setupWbc() || !this->setupStateEstimate())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // check the joint list against the model and build the joint order maps; the mapping is
        // logged once to check the order
        std::string error;
        if (!this->joint_map_.build(this->robotModel().model(), this->params_.joints, error))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "on_configure: %s", error.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_node()->get_logger(), "Joint map (ros2_control -> Pinocchio):%s",
                    this->joint_map_.describe(this->params_.joints).c_str());

        // ros2_control interfaces: names to claim now, bound in on_activate
        this->robot_ = std::make_unique<RobotInterface>(this->params_, this->joint_map_);

        // per-cycle buffers with their final sizes: update() never allocates
        this->AllocateBuffers();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WBCController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        const auto logger = get_node()->get_logger();
        std::string error;

        // find the loaned interfaces (joints, IMU, motors enabled) by name
        if (!this->robot_->bind(this->command_interfaces_, this->state_interfaces_, error))
        {
            RCLCPP_ERROR(logger, "on_activate: %s", error.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // refuse to activate with the motors off (the WBC would command torques the drives do not apply)
        if (!this->robot_->motorsEnabled())
        {
            RCLCPP_ERROR(logger, "on_activate: motors are disabled ('%s' = 0); enable the hardware first",
                         this->params_.enabled_interface.c_str());
            this->robot_->release();
            return controller_interface::CallbackReturn::ERROR;
        }

        // bumpless hold: keep the command the drives already hold (see RobotInterface)
        if (!this->robot_->holdCurrentCommand(error))
        {
            RCLCPP_ERROR(logger, "on_activate: %s", error.c_str());
            this->robot_->release();
            return controller_interface::CallbackReturn::ERROR;
        }

        // first reading, to check the IMU once (standing still: quaternion about (1, 0, 0, 0),
        // acceleration about (0, 0, +9.81))
        this->robot_->read(this->sensor_);
        const auto &imu = this->sensor_.imu_sensor;
        RCLCPP_INFO(logger, "IMU '%s': orientation (w x y z) %.3f %.3f %.3f %.3f, acceleration %.2f %.2f %.2f",
                    this->params_.imu_name.c_str(), imu.imu_quat_.w(), imu.imu_quat_.x(), imu.imu_quat_.y(),
                    imu.imu_quat_.z(), imu.imu_accel_L(0), imu.imu_accel_L(1), imu.imu_accel_L(2));

        // estimators and planner start from the current state
        this->resetModules();

        RCLCPP_INFO(logger, "Active: %zu joints claimed (position / velocity / effort), holding the current pose",
                    this->params_.joints.size());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WBCController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // drop the references to the loans before the controller manager takes them back; the last
        // command stays in the interfaces, the next controller takes it over bumplessly
        this->robot_->release();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type WBCController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)period;
        const auto logger = get_node()->get_logger();

        // motors disabled (hardware fault / command timeout): no control, hold at the measured
        // position so there is no jump once the motors are enabled again
        if (!this->robot_->motorsEnabled())
        {
            RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000,
                                  "motors disabled ('%s' = 0): holding the measured pose", this->params_.enabled_interface.c_str());
            this->robot_->holdMeasured();
            return controller_interface::return_type::OK;
        }

        // Read robot feedback measurement (IMU + joint states, Pinocchio order); the hardware read()
        // was already called by the controller manager this cycle. Invalid reading -> last command
        if (!this->robot_->read(this->sensor_))
        {
            RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000,
                                  "invalid sensor reading (NaN / Inf): keeping the last command");
            this->robot_->writeLastCommand();
            return controller_interface::return_type::OK;
        }

        // Get joystick command (B7)

        // estimate base state and contact state
        this->estimate();

        // compute the joint command (planner + TSID, commands 1-5) and send it; no command yet
        // (B2: nothing computed) -> keep holding the last one, so the drives get a fresh command
        // every cycle (hardware command timeout)
        if (this->computeCommand(time))
        {
            this->robot_->write(this->q_cmd_, this->v_cmd_, this->tau_cmd_);
        }
        else
        {
            this->robot_->writeLastCommand();
        }

        return controller_interface::return_type::OK;
    }

    // ---------------------------------------------------------------------
    // on_configure: QP_TSID (loads the URDF as a free-flyer robot, the TSID tasks and contacts) and
    // the LIP task planner, both from the config yaml; control period dt_ of all modules.
    // Returns false (error logged) if a file or value is invalid (the constructors throw).
    // ---------------------------------------------------------------------
    bool WBCController::setupWbc ()
    {
        const auto logger = get_node()->get_logger();
        try
        {
            // QP_TSID: owns the robot model used by every module
            this->qp_tsid_controller_ = std::make_unique<QP_TSID>(this->params_.config_path, this->params_.urdf_path);
            const tsid::robots::RobotWrapper &robot = this->robotModel();
            RCLCPP_INFO(logger, "Robot model: nq %d, nv %d, na %d", robot.nq(), robot.nv(), robot.na());

            // LIP task planner: section "LIPPlanner" of the same yaml
            this->lip_task_planner_ = std::make_unique<LIPTaskPlanner>(this->params_.config_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "setupWbc: %s", e.what());
            return false;
        }

        // control period of all modules: sampling_frequency of the yaml. get_update_rate() is 0
        // unless the controller has its own update_rate parameter (0 = every controller_manager
        // cycle, whose rate the controller cannot read), so it is only used as a cross-check.
        // sampling_frequency must equal controller_manager.update_rate (controllers.yaml).
        this->dt_ = this->lip_task_planner_->getDt();
        if (get_update_rate() > 0 && std::abs(this->dt_ * get_update_rate() - 1.0) > 1e-6)
        {
            RCLCPP_WARN(logger, "LIPPlanner sampling_frequency (%.0f Hz) differs from the controller update rate "
                        "(%u Hz): the plan and the estimators run at the wrong speed",
                        1.0 / this->dt_, get_update_rate());
        }
        return true;
    }

    // ---------------------------------------------------------------------
    // on_configure, after setupWbc: state estimator and contact fusion on the robot model of
    // QP_TSID, at the control period dt_.
    // Returns false (error logged) if a frame is invalid (the constructors throw).
    // ---------------------------------------------------------------------
    bool WBCController::setupStateEstimate ()
    {
        const auto logger = get_node()->get_logger();
        const tsid::robots::RobotWrapper &robot = this->robotModel();
        const std::string feet[2] = {this->params_.left_foot_frame, this->params_.right_foot_frame};
        try
        {
            // state estimator: base position / velocity from IMU + leg kinematics
            this->state_estimator_ = std::make_unique<StateEstimator>(robot, this->dt_, feet[0], feet[1]);

            // contact fusion: foot contact probability from the momentum observer, the foot height
            // and the gait phase (observer cutoff 15 Hz)
            this->contact_fusion_ = std::make_unique<ContactFusion>(
                robot.model(), std::vector<std::string>{feet[0], feet[1]}, this->dt_, 15.0);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "setupStateEstimate: %s", e.what());
            return false;
        }

        // sole corners of each foot: bottom face of the foot's collision box in the URDF (the same
        // source as the TSID contacts), for the foot height of the contact fusion
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

        RCLCPP_INFO(logger, "Control modules ready: QP_TSID, LIPTaskPlanner, StateEstimator, ContactFusion (dt %.4f s)",
                    this->dt_);
        return true;
    }

    // ---------------------------------------------------------------------
    // on_activate: start the estimators and the planner from the current state (sensor_ holds a
    // first reading).
    // ---------------------------------------------------------------------
    void WBCController::resetModules ()
    {
        // contact probability: the robot starts standing on both feet (same as the initial state of
        // the contact fusion); used by the state estimator in the first update()
        this->contact_probability_.setOnes();

        // TODO (B3): estimator base position from leg kinematics (setBasePosEst), contact fusion
        // reset; planner reset(com, left foot, right foot) from the estimated state
    }

    // ---------------------------------------------------------------------
    // update(): base state and contact state of this cycle. Real-time.
    // ---------------------------------------------------------------------
    void WBCController::estimate ()
    {
        // TODO (B3):
        //   this->state_estimator_->update(this->sensor_, this->contact_probability_);
        //   this->state_estimator_->getPinocchioState(this->q_, this->v_);
        //   contact fusion with the last torque command -> contact_probability_ (used next cycle)
    }

    // ---------------------------------------------------------------------
    // update(): joint command from the estimated state (Pinocchio order). Real-time.
    // Returns false while there is no command yet: the last one is held.
    // ---------------------------------------------------------------------
    bool WBCController::computeCommand (const rclcpp::Time &time)
    {
        (void)time;

        // set task based on joystick command, then process the task:
        //   command 1: homing, postural task only, control all the joints to the home position (B6)
        //   command 2: init walking, lower the CoM to the reference zc (bend the knees) (B8)
        //   command 3: walking with constant step length and swing time (B10)
        //   command 4: heading turning (B11)
        //   command 5: transition from walking to stop (B11)
        // each fills q_cmd_ / v_cmd_ / tau_cmd_ from the TSID solution (dv_, tau_) and returns true

        return false;  // B2: no command yet, hold
    }

    // ---------------------------------------------------------------------
    // Sizes the per-cycle buffers once (called from on_configure, after setupWbc and
    // joint_map_.build). update() only writes into them: an Eigen::VectorXd that is created or
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

        // joint command, Pinocchio order (robot_->write converts it to ros2_control order)
        this->q_cmd_ = Eigen::VectorXd::Zero(na);
        this->v_cmd_ = Eigen::VectorXd::Zero(na);
        this->tau_cmd_ = Eigen::VectorXd::Zero(na);
    }

} // namespace biped_controllers

PLUGINLIB_EXPORT_CLASS(biped_controllers::WBCController, controller_interface::ControllerInterface)
