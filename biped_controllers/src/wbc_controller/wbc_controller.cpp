#include "wbc_controller/wbc_controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "biped_wbc/common/urdf_geometry.hpp"
#include "biped_wbc/common/utils.hpp"   // computeIK_Leg, quinticTrajectory
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>

namespace biped_controllers
{
    // interfaces the controller claims (called after on_configure, before on_activate): the names
    // come from RobotInterface (joint PVT commands; joint states, IMU, motors enabled)
    controller_interface::InterfaceConfiguration WBCController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        if (this->robot_ros_interface_)
        {
            config.names = this->robot_ros_interface_->commandInterfaceNames();
        }
        return config;
    }

    controller_interface::InterfaceConfiguration WBCController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        if (this->robot_ros_interface_)
        {
            config.names = this->robot_ros_interface_->stateInterfaceNames();
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

        // robot model from the URDF, shared by every module
        if (!this->setupRobotModel())
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

        // control modules on that model: QP_TSID + planner, then the estimators
        if (!this->setupWbc() || !this->setupStateEstimate())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // ros2_control interfaces: names to claim now, bound in on_activate
        this->robot_ros_interface_ = std::make_unique<RobotInterface>(this->params_, this->joint_map_);

        // per-cycle buffers with their final sizes: update() never allocates
        this->AllocateBuffers();

        // debug topic of the estimate
        this->setupDebugPublisher();

        // motion commands from the keyboard / joystick
        this->setupCommandSubscriber();

        // simulator ground truth instead of the base estimate (simulation only)
        this->setupGroundTruthSubscriber();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WBCController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        const auto logger = get_node()->get_logger();
        std::string error;

        // find the loaned interfaces (joints, IMU, motors enabled) by name
        if (!this->robot_ros_interface_->bind(this->command_interfaces_, this->state_interfaces_, error))
        {
            RCLCPP_ERROR(logger, "on_activate: %s", error.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // refuse to activate with the motors off (the WBC would command torques the drives do not apply)
        if (!this->robot_ros_interface_->motorsEnabled())
        {
            RCLCPP_ERROR(logger, "on_activate: motors are disabled ('%s' = 0); enable the hardware first",
                         this->params_.enabled_interface.c_str());
            this->robot_ros_interface_->release();
            return controller_interface::CallbackReturn::ERROR;
        }

        // bumpless hold: keep the command the drives already hold (see RobotInterface)
        if (!this->robot_ros_interface_->holdCurrentCommand(error))
        {
            RCLCPP_ERROR(logger, "on_activate: %s", error.c_str());
            this->robot_ros_interface_->release();
            return controller_interface::CallbackReturn::ERROR;
        }

        // first reading: starting state of the estimators; IMU logged once to check it (standing
        // still: quaternion about (1, 0, 0, 0), acceleration about (0, 0, +9.81))
        if (!this->robot_ros_interface_->read(this->sensor_))
        {
            RCLCPP_ERROR(logger, "on_activate: invalid sensor reading (NaN / Inf)");
            this->robot_ros_interface_->release();
            return controller_interface::CallbackReturn::ERROR;
        }
        const auto &imu = this->sensor_.imu_sensor;
        RCLCPP_INFO(logger, "IMU '%s': orientation (w x y z) %.3f %.3f %.3f %.3f, acceleration %.2f %.2f %.2f",
                    this->params_.imu_name.c_str(), imu.imu_quat_.w(), imu.imu_quat_.x(), imu.imu_quat_.y(),
                    imu.imu_quat_.z(), imu.imu_accel_L(0), imu.imu_accel_L(1), imu.imu_accel_L(2));

        // estimators, TSID and planner start from the current state
        this->resetModules();
        this->debug_counter_ = 0;

        // targets of commands 1 / 2 (IK once), references held at the current state
        if (!this->setupPostureTargets())
        {
            this->robot_ros_interface_->release();
            return controller_interface::CallbackReturn::ERROR;
        }

        // a command sent before this activation is not replayed
        this->last_command_seq_ = this->command_buffer_.readFromNonRT()->seq;

        RCLCPP_INFO(logger, "Active: %zu joints claimed (position / velocity / effort), holding the current pose",
                    this->params_.joints.size());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WBCController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // drop the references to the loans before the controller manager takes them back; the last
        // command stays in the interfaces, the next controller takes it over bumplessly
        this->robot_ros_interface_->release();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type WBCController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)period;
        const auto logger = get_node()->get_logger();

        // motors disabled (hardware fault / command timeout): no control, hold at the measured
        // position so there is no jump once the motors are enabled again
        if (!this->robot_ros_interface_->motorsEnabled())
        {
            RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000,
                                  "motors disabled ('%s' = 0): holding the measured pose", this->params_.enabled_interface.c_str());
            this->robot_ros_interface_->holdMeasured();
            return controller_interface::return_type::OK;
        }

        // Read robot feedback measurement (IMU + joint states, Pinocchio order); the hardware read()
        // was already called by the controller manager this cycle. Invalid reading -> last command
        if (!this->robot_ros_interface_->read(this->sensor_))
        {
            RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000,
                                  "invalid sensor reading (NaN / Inf): keeping the last command");
            this->robot_ros_interface_->writeLastCommand();
            return controller_interface::return_type::OK;
        }

        // estimate base state and contact state
        this->estimate();
        this->publishDebug();

        // joystick / keyboard command (B7): may start a transition from the state just estimated
        this->handleMotionCommand(time);

        // compute the joint command (planner + TSID, commands 1-5) and send it; no command yet
        // (B2: nothing computed) -> keep holding the last one, so the drives get a fresh command
        // every cycle (hardware command timeout)
        if (this->computeCommand(time))
        {
            this->robot_ros_interface_->write(this->q_cmd_, this->v_cmd_, this->tau_cmd_);
        }
        else
        {
            this->robot_ros_interface_->writeLastCommand();
        }

        return controller_interface::return_type::OK;
    }

    // ---------------------------------------------------------------------
    // on_configure: loads the URDF as a free-flyer robot model (base pose + joints), the one model
    // every module uses. Returns false (error logged) if the URDF cannot be loaded.
    // ---------------------------------------------------------------------
    bool WBCController::setupRobotModel ()
    {
        const auto logger = get_node()->get_logger();
        try
        {
            // no package directories: the robot model needs no meshes
            const std::vector<std::string> package_dirs;
            this->robot_pin_wrapper_ = std::make_unique<tsid::robots::RobotWrapper>(
                this->params_.urdf_path, package_dirs, pinocchio::JointModelFreeFlyer(), false);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(logger, "setupRobotModel: %s", e.what());
            return false;
        }
        const tsid::robots::RobotWrapper &robot = this->robotModel();
        RCLCPP_INFO(logger, "Robot model: nq %d, nv %d, na %d", robot.nq(), robot.nv(), robot.na());
        return true;
    }

    // ---------------------------------------------------------------------
    // on_configure: QP_TSID (TSID tasks and contacts) on the robot model and the LIP task planner,
    // both from the config yaml; control period dt_ of all modules.
    // Returns false (error logged) if a file or value is invalid (the constructors throw).
    // ---------------------------------------------------------------------
    bool WBCController::setupWbc ()
    {
        const auto logger = get_node()->get_logger();
        try
        {
            // QP_TSID on the shared robot model; the URDF for the sole geometry of the contacts
            this->qp_tsid_controller_ = std::make_unique<QP_TSID>(this->robotModel(), this->params_.config_path,
                                                                  this->params_.urdf_path);

            // walk planner: section "LIPPlanner" of the same yaml, support frames on the foot frames
            this->walk_planner_ = std::make_unique<WalkPlanner>(this->robotModel(), this->params_.config_path,
                                                                this->params_.left_foot_frame,
                                                                this->params_.right_foot_frame);
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
        this->dt_ = this->walk_planner_->getDt();
        if (get_update_rate() > 0 && std::abs(this->dt_ * get_update_rate() - 1.0) > 1e-6)
        {
            RCLCPP_WARN(logger, "LIPPlanner sampling_frequency (%.0f Hz) differs from the controller update rate "
                        "(%u Hz): the plan and the estimators run at the wrong speed",
                        1.0 / this->dt_, get_update_rate());
        }
        return true;
    }

    // ---------------------------------------------------------------------
    // on_configure, after setupWbc (needs dt_): state estimator and contact fusion on the robot
    // model, at the control period dt_.
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
            if (utils::soleCornersFromUrdf(this->params_.urdf_path, feet[i], Eigen::Vector3d::UnitZ(), corners))
            {
                this->contact_fusion_->setSoleCorners(i, corners);
            }
            else
            {
                RCLCPP_WARN(logger, "No sole box in the URDF for foot frame '%s': the contact fusion uses the "
                            "frame origin as the sole", feet[i].c_str());
            }
        }

        RCLCPP_INFO(logger, "Control modules ready: QP_TSID, WalkPlanner, StateEstimator, ContactFusion (dt %.4f s)",
                    this->dt_);
        return true;
    }

    // ---------------------------------------------------------------------
    // on_activate: start the estimators and the planner from the current state (sensor_ holds a
    // first reading).
    // ---------------------------------------------------------------------
    void WBCController::resetModules ()
    {
        // state estimator: standing on both feet, base above the origin at the height given by the
        // leg kinematics, feet on the ground
        this->state_estimator_->reset(this->sensor_);

        // contact fusion: both feet in contact, observer restarted (also after a re-activation)
        this->contact_fusion_->reset();

        // contact probability: both feet (same as the contact fusion after reset); used by the
        // state estimator in the first update()
        this->contact_probability_.setOnes();

        // TSID: every reference at the estimated state, both feet in contact where they are now
        this->state_estimator_->getPinocchioState(this->q_, this->v_);
        if (this->params_.use_ground_truth && !this->applyGroundTruth())
        {
            RCLCPP_WARN(get_node()->get_logger(), "use_ground_truth: no message on '%s' yet, starting from the estimate",
                        this->params_.ground_truth_topic.c_str());
        }
        this->qp_tsid_controller_->reset(this->q_, this->v_);

        // TODO (B6): planner reset(com, left foot, right foot) from the estimated state
    }

    // ---------------------------------------------------------------------
    // update(): base state and contact state of this cycle. Real-time.
    // ---------------------------------------------------------------------
    void WBCController::estimate ()
    {
        // base state: IMU + leg kinematics of the legs in contact. Contact = the contact in effect
        // (scheduled_contact_ of the last cycle, 1 stance / 0 swing: the planner's phase, and with
        // use_time_base false the swing foot from its detected touchdown on), not the fused contact
        // probability below (it lags the liftoff: the lifting leg's kinematics would still be trusted)
        const Eigen::Vector2d planned_contact(this->scheduled_contact_[0] ? 1.0 : 0.0,
                                              this->scheduled_contact_[1] ? 1.0 : 0.0);
        this->state_estimator_->update(this->sensor_, planned_contact);
        this->state_estimator_->getPinocchioState(this->q_, this->v_);
        if (this->params_.use_ground_truth && !this->applyGroundTruth())
        {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                 "use_ground_truth: no message on '%s': using the estimate",
                                 this->params_.ground_truth_topic.c_str());
        }

        // contact state: momentum observer with the measured joint torques -> foot forces, then
        // fused with the foot height and the planned contact -> probability for the next cycle
        this->contact_fusion_->computeDisturbanceTorque(this->q_, this->v_, this->sensor_.actuator_state.torquej);
        this->contact_probability_ =
            this->contact_fusion_->fuseContactProbability(this->scheduled_contact_, this->contact_phase_);

    }

    // ---------------------------------------------------------------------
    // update(): joint command from the estimated state (Pinocchio order). Real-time.
    // Returns false while there is no command yet: the last one is held.
    // ---------------------------------------------------------------------
    bool WBCController::computeCommand (const rclcpp::Time &time)
    {
        // set task based on joystick command, then process the task:
        //   command 1: homing, postural task only, control all the joints to the home position (B6)
        //   command 2: init walking, lower the CoM to the reference zc (bend the knees) (B8)
        //   command 3: walking with constant step length and swing time (B10)
        //   command 4: heading turning (B11)
        //   command 5: transition from walking to stop (B11)
        // each fills q_cmd_ / v_cmd_ / tau_cmd_ from the TSID solution (dv_, tau_) and returns true

        // walking (command 3): planner references
        if (this->mode_ == Mode::WALK)
        {
            return this->computeWalkCommand(time);
        }

        // commands 1 / 2, a transition from the last reference to the target of the
        // command (handleMotionCommand); after it (or before any command) the target is held
        const double t = time.seconds() - this->transition_start_time_;

        // joint reference of this cycle: quintic to the joint target
        utils::quinticTrajectory(this->qj_start_, this->qj_target_, t, kTransitionDuration, this->qj_ref_,
                                 this->dqj_ref_, this->ddqj_ref_);
        this->qp_tsid_controller_->setPostureReference(this->qj_ref_, this->dqj_ref_, this->ddqj_ref_);

        // CoM reference of this cycle: same timing, to the CoM of the target pose
        utils::quinticTrajectory(this->com_start_, this->com_target_, t, kTransitionDuration, this->com_ref_,
                                 this->dcom_ref_, this->ddcom_ref_);
        this->qp_tsid_controller_->setCoMReference(this->com_ref_, this->dcom_ref_, this->ddcom_ref_);

        // TSID: accelerations and torques for it with both feet in contact; failed -> hold
        if (!this->qp_tsid_controller_->solve(time.seconds(), this->q_, this->v_))
        {
            RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                  "TSID QP failed (t %.3f s): keeping the last command", time.seconds());
            return false;
        }
        this->dv_ = this->qp_tsid_controller_->getAccelerations();
        this->tau_ = this->qp_tsid_controller_->getTorques();

        // drive command: the motor PD tracks the joint reference, the TSID torque is the feedforward
        this->q_cmd_ = this->qj_ref_;
        this->v_cmd_ = this->dqj_ref_;
        this->tau_cmd_ = this->tau_;
        return true;
    }

    // ---------------------------------------------------------------------
    // on_activate: targets of commands 1 / 2 and every reference at the current state. Not
    // real-time (IK, allocates).
    //   command 1 (homing): all joints 0
    //   command 2 (init walking): IK once, both feet keep their pose relative to the base, only
    //   lower (base kStandBaseHeight above the ground)
    // The CoM of each target pose is kept in the left foot frame: startTransition places it on the
    // foot where it is then (the feet do not move in these commands).
    // ---------------------------------------------------------------------
    bool WBCController::setupPostureTargets ()
    {
        const auto logger = get_node()->get_logger();
        const pinocchio::Model &model = this->robotModel().model();
        pinocchio::Data data(model);
        const int na = this->robotModel().na();

        // base and foot poses of the estimated state (q_ from resetModules)
        pinocchio::framesForwardKinematics(model, data, this->q_);
        const pinocchio::SE3 oMb = data.oMi[1]; // free-flyer joint = base (copy: the IK below reuses data)
        const pinocchio::SE3 oMf[2] = {data.oMf[model.getFrameId(this->params_.left_foot_frame)],
                                       data.oMf[model.getFrameId(this->params_.right_foot_frame)]};

        // command 2: IK target relative to the base, same foot x, y and orientation; foot z relative
        // to the base = foot z in the world - desired base height (ground at z = 0 in the estimator)
        pinocchio::SE3 bMf[2];
        for (int i = 0; i < 2; ++i)
        {
            bMf[i] = oMb.actInv(oMf[i]);
            bMf[i].translation().z() = oMf[i].translation().z() - kStandBaseHeight;
        }
        const utils::IkRes ik = utils::computeIK_Leg(model, data, bMf[0].rotation(), bMf[0].translation(),
                                                     bMf[1].rotation(), bMf[1].translation());
        if (ik.status != 0)
        {
            RCLCPP_ERROR(logger, "init walking: IK for base height %.3f m did not converge (error %.2e)",
                         kStandBaseHeight, ik.err.norm());
            return false;
        }
        this->qj_stand_ = ik.jointPosRes;

        // command 1: all joints 0
        this->qj_home_ = Eigen::VectorXd::Zero(na);

        // CoM of each target pose in the left foot frame (base pose does not matter: identity)
        Eigen::VectorXd q_target = pinocchio::neutral(model);
        const Eigen::VectorXd *qj_targets[2] = {&this->qj_home_, &this->qj_stand_};
        Eigen::Vector3d *com_in_foot[2] = {&this->com_home_in_foot_, &this->com_stand_in_foot_};
        for (int k = 0; k < 2; ++k)
        {
            q_target.tail(na) = *qj_targets[k];
            const Eigen::Vector3d com = pinocchio::centerOfMass(model, data, q_target);
            pinocchio::framesForwardKinematics(model, data, q_target);
            *com_in_foot[k] = data.oMf[this->left_foot_id_].actInv(com);
        }

        // every reference at the current state, held until the first command (start = target)
        this->qj_start_ = this->q_.tail(na);
        this->qj_target_ = this->qj_start_;
        this->qj_ref_ = this->qj_start_;
        this->dqj_ref_ = Eigen::VectorXd::Zero(na);
        this->ddqj_ref_ = Eigen::VectorXd::Zero(na);
        this->com_start_ = pinocchio::centerOfMass(model, data, this->q_);
        this->com_target_ = this->com_start_;
        this->com_ref_ = this->com_start_;
        this->dcom_ref_ = Eigen::VectorXd::Zero(3);
        this->ddcom_ref_ = Eigen::VectorXd::Zero(3);
        this->transition_start_time_ = -std::numeric_limits<double>::infinity();
        this->mode_ = Mode::STAND;
        this->in_walking_pose_ = false;

        RCLCPP_INFO(logger, "motion targets: base now %.3f m; homing: joints 0; init walking: base %.3f m "
                    "(knee %.3f / %.3f rad); both over %.1f s", oMb.translation().z(), kStandBaseHeight,
                    this->qj_stand_(3), this->qj_stand_(9), kTransitionDuration);
        return true;
    }

    // ---------------------------------------------------------------------
    // on_configure: subscription of the motion commands. The callback (executor thread) only
    // stores the command with a count; update() reads it without waiting (RealtimeBuffer).
    // ---------------------------------------------------------------------
    void WBCController::setupCommandSubscriber ()
    {
        // created once (on_configure may run again after a cleanup)
        if (this->command_sub_)
        {
            return;
        }
        this->command_sub_ = get_node()->create_subscription<humanoid_mpc_msgs::msg::MotionCommand>(
            "/humanoid/motion_command", rclcpp::QoS(10),
            [this](const humanoid_mpc_msgs::msg::MotionCommand::SharedPtr msg)
            {
                MotionCommandSample sample;
                sample.command = msg->command;
                sample.seq = ++this->command_seq_;
                this->command_buffer_.writeFromNonRT(sample);
            });
    }

    // ---------------------------------------------------------------------
    // on_configure, use_ground_truth only: the callback (executor thread) only stores the message;
    // estimate() reads it without waiting (RealtimeBuffer).
    // ---------------------------------------------------------------------
    void WBCController::setupGroundTruthSubscriber ()
    {
        // created once (on_configure may run again after a cleanup)
        if (!this->params_.use_ground_truth || this->ground_truth_sub_)
        {
            return;
        }
        this->ground_truth_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
            this->params_.ground_truth_topic, rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->ground_truth_buffer_.writeFromNonRT(msg); });
    }

    // ---------------------------------------------------------------------
    // Base state from the ground truth. MuJoCo free joint (mujoco_ros2_control): position and
    // linear velocity in the world frame, angular velocity in the body frame; Pinocchio wants both
    // velocities in the base frame. Real-time.
    // ---------------------------------------------------------------------
    bool WBCController::applyGroundTruth ()
    {
        const std::shared_ptr<nav_msgs::msg::Odometry> &msg = *this->ground_truth_buffer_.readFromRT();
        if (!msg)
        {
            return false;
        }
        const auto &p = msg->pose.pose.position;
        const auto &o = msg->pose.pose.orientation;
        const auto &lin = msg->twist.twist.linear;
        const auto &ang = msg->twist.twist.angular;
        const Eigen::Quaterniond quat(o.w, o.x, o.y, o.z);

        // pose: position, quaternion (x y z w)
        this->q_.head<3>() << p.x, p.y, p.z;
        this->q_.segment<4>(3) << o.x, o.y, o.z, o.w;

        // twist in the base frame: linear rotated from the world frame, angular as is
        this->v_.head<3>().noalias() = quat.toRotationMatrix().transpose() * Eigen::Vector3d(lin.x, lin.y, lin.z);
        this->v_.segment<3>(3) << ang.x, ang.y, ang.z;
        return true;
    }

    // ---------------------------------------------------------------------
    // update(): starts the transition of a new motion command. Real-time.
    // ---------------------------------------------------------------------
    void WBCController::handleMotionCommand (const rclcpp::Time &time)
    {
        // nothing new since the last cycle
        const MotionCommandSample &cmd = *this->command_buffer_.readFromRT();
        if (cmd.seq == this->last_command_seq_)
        {
            return;
        }
        this->last_command_seq_ = cmd.seq;
        const auto logger = get_node()->get_logger();
        const bool transition_running = time.seconds() - this->transition_start_time_ < kTransitionDuration;

        // walking: every message keeps it going; the first one at rest starts it
        if (cmd.command == humanoid_mpc_msgs::msg::MotionCommand::WALKING)
        {
            this->walk_cmd_time_ = time.seconds();
            if (this->mode_ == Mode::WALK)
            {
                return;
            }
            if (transition_running || !this->in_walking_pose_)
            {
                RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 2000,
                                     "walking ignored: %s", transition_running ? "a transition is running"
                                                                               : "send init walking first");
                return;
            }
            this->startWalking(time);
            return;
        }
        if (this->mode_ == Mode::WALK)
        {
            RCLCPP_WARN(logger, "motion command %u ignored while walking", cmd.command);
            return;
        }

        // one transition at a time: the next one starts from a target, at rest
        if (transition_running)
        {
            RCLCPP_WARN(logger, "motion command %u ignored: a transition is running", cmd.command);
            return;
        }

        // target of the command
        switch (cmd.command)
        {
        case humanoid_mpc_msgs::msg::MotionCommand::HOMING:
            this->startTransition(this->qj_home_, this->com_home_in_foot_, time);
            this->in_walking_pose_ = false;
            RCLCPP_INFO(logger, "command 1: homing (joints to 0) over %.1f s", kTransitionDuration);
            break;
        case humanoid_mpc_msgs::msg::MotionCommand::INIT_WALKING:
            this->startTransition(this->qj_stand_, this->com_stand_in_foot_, time);
            this->in_walking_pose_ = true;
            RCLCPP_INFO(logger, "command 2: init walking (base to %.3f m) over %.1f s", kStandBaseHeight,
                        kTransitionDuration);
            break;
        default:
            RCLCPP_WARN(logger, "unknown motion command %u ignored", cmd.command);
            break;
        }
    }

    // ---------------------------------------------------------------------
    // update(): new transition from the references of this cycle (no jump in the command) to the
    // joint target and the CoM of that pose on the left foot. Real-time: writes into the buffers.
    // ---------------------------------------------------------------------
    void WBCController::startTransition (const Eigen::VectorXd &qj_target, const Eigen::Vector3d &com_in_foot,
                                         const rclcpp::Time &time)
    {
        const pinocchio::Model &model = this->robotModel().model();

        // left foot pose of the estimated state (FK on the preallocated data_)
        pinocchio::forwardKinematics(model, this->data_, this->q_);
        const pinocchio::SE3 &oMf = pinocchio::updateFramePlacement(model, this->data_, this->left_foot_id_);

        // start at the current references, end at the target
        this->qj_start_ = this->qj_ref_;
        this->qj_target_ = qj_target;
        this->com_start_ = this->com_ref_;
        this->com_target_ = oMf.act(com_in_foot);
        this->transition_start_time_ = time.seconds();
    }

    // ---------------------------------------------------------------------
    // handleMotionCommand: start walking from the current state (standing at rest, init walking
    // pose, both feet in contact).
    // ---------------------------------------------------------------------
    void WBCController::startWalking (const rclcpp::Time &time)
    {
        // planner from the estimated state: support frame on the left ankle, LIP at the CoM height
        this->walk_planner_->reset(this->q_);
        this->prev_swing_foot_ = -1; // touchdown detection: no step yet
        this->liftoff_seen_ = false;
        this->landed_ = false;

        // base upright: roll = pitch = yaw = 0 (translation masked out in the task)
        this->qp_tsid_controller_->setBaseReference(pinocchio::SE3::Identity(), this->zero6_, this->zero6_);

        // joint command: from the last one until the first walk cycle (the posture reference stays
        // the stand pose)
        this->q_des_ = this->q_cmd_;
        this->v_des_ = this->v_cmd_;

        this->mode_ = Mode::WALK;
        this->walk_cmd_time_ = time.seconds();
        this->stop_requested_ = false;
        RCLCPP_INFO(get_node()->get_logger(), "command 3: walking at %.3f m/s (CoM height %.3f m)",
                    this->params_.walk_vx, this->walk_planner_->getRefCoMPosition().z());
    }

    // ---------------------------------------------------------------------
    // computeCommand while walking. Real-time except the contact switches (TSID adds / removes a
    // task: allocates, twice per step).
    // ---------------------------------------------------------------------
    bool WBCController::computeWalkCommand (const rclcpp::Time &time)
    {
        const auto logger = get_node()->get_logger();
        const int na = this->robotModel().na();

        // key released (no WALKING message): finish the step, then stop and rest
        if (!this->stop_requested_ && time.seconds() - this->walk_cmd_time_ > kWalkTimeout)
        {
            this->walk_planner_->requestStop();
            this->stop_requested_ = true;
            RCLCPP_INFO(logger, "walk command released: stopping");
        }

        // planner: one cycle at the walking speed of the yaml (walk_vx, 0 = stepping in place). Its
        // timing (CoM plan, support switch) is time based in both modes
        this->walk_planner_->setVelocityCommand(this->params_.walk_vx);
        this->walk_planner_->planWalking(this->q_);
        const bool swinging = this->walk_planner_->isSwinging();
        const int swing_foot = this->walk_planner_->getSwingFoot();

        // early touchdown (use_time_base false): the contact estimator (estimate() of this cycle)
        // has seen the swing foot off the ground in this step and now on it, past kMinTouchdownPhase.
        // The foot lands in TSID at once and stays down until the planner's step ends; a late
        // touchdown lands at the end of the step (Tswing, the time-out)
        if (swinging && swing_foot != this->prev_swing_foot_)
        {
            this->liftoff_seen_ = false; // new step
            this->landed_ = false;
        }
        if (!this->params_.use_time_base && swinging && !this->landed_)
        {
            const bool contact = this->contact_fusion_->getContactState(swing_foot);
            this->liftoff_seen_ = this->liftoff_seen_ || !contact;
            if (this->liftoff_seen_ && contact && this->walk_planner_->getPhase() >= kMinTouchdownPhase)
            {
                this->landed_ = true;
                RCLCPP_INFO(logger, "touchdown of the %s foot detected at step phase %.2f: contact on",
                            swing_foot == 0 ? "left" : "right", this->walk_planner_->getPhase());
            }
        }
        this->prev_swing_foot_ = swinging ? swing_foot : -1;
        const bool in_swing = swinging && !this->landed_;

        // contacts: the plan, the swing foot down once landed; land first, then lift with its
        // swing reference set
        for (int i = 0; i < 2; ++i)
        {
            const bool in_contact = !(in_swing && swing_foot == i);
            if (in_contact && !this->qp_tsid_controller_->isContactActive(i))
            {
                this->qp_tsid_controller_->landFoot(i);
            }
            this->scheduled_contact_[i] = in_contact;
            this->contact_phase_[i] = this->walk_planner_->getPhase();
        }
        if (in_swing)
        {
            this->swing_pose_.rotation() = this->walk_planner_->getRefSwingFootRotation();
            this->swing_pose_.translation() = this->walk_planner_->getRefSwingFootPosition();
            this->swing_vel_.head<3>() = this->walk_planner_->getRefSwingFootVelocity();
            this->swing_vel_.tail<3>().setZero();
            this->swing_acc_.head<3>() = this->walk_planner_->getRefSwingFootAcceleration();
            this->swing_acc_.tail<3>().setZero();
            this->qp_tsid_controller_->setSwingReference(swing_foot, this->swing_pose_, this->swing_vel_, this->swing_acc_);
            if (this->qp_tsid_controller_->isContactActive(swing_foot))
            {
                this->qp_tsid_controller_->liftFoot(swing_foot);
            }
        }

        // CoM reference of the planner (copied into the size-3 buffers: no temporary)
        this->com_ref_ = this->walk_planner_->getRefCoMPosition();
        this->dcom_ref_ = this->walk_planner_->getRefCoMVelocity();
        this->ddcom_ref_ = this->walk_planner_->getRefCoMAcceleration();
        this->qp_tsid_controller_->setCoMReference(this->com_ref_, this->dcom_ref_, this->ddcom_ref_);

        // QP solver
        // TSID with the contacts of the plan; failed -> hold
        if (!this->qp_tsid_controller_->solve(time.seconds(), this->q_, this->v_))
        {
            RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000,
                                  "TSID QP failed while walking (t %.3f s): keeping the last command", time.seconds());
            return false;
        }

        // Output acceleration and torque cmd
        this->dv_ = this->qp_tsid_controller_->getAccelerations();
        this->tau_ = this->qp_tsid_controller_->getTorques();

        // drive command: the measured joint state, so the drive PD is zero at the read instant and
        // the TSID torque does the work (as torque control). Not q + v dt: kp * v * dt pushes along
        // the joint velocity (negative damping), large on the light swing ankle
        this->q_des_ = this->q_.tail(na);
        this->v_des_ = this->v_.tail(na);
        this->q_cmd_ = this->q_des_;
        this->v_cmd_ = this->v_des_;
        this->tau_cmd_ = this->tau_;

        // at rest (double support) once the planned CoM has reached its CP (the middle of the feet,
        // it converges after the stop phase): standing again, every reference held where it is (the
        // next init walking / homing starts from here)
        const Eigen::Vector2d com_to_cp = this->walk_planner_->getRefCP() - this->com_ref_.head<2>();
        if (this->walk_planner_->getWalkPhase() == WalkPhase::REST && com_to_cp.norm() < kRestTolerance)
        {
            this->mode_ = Mode::STAND;
            this->qj_ref_ = this->q_des_;
            this->qj_start_ = this->q_des_;
            this->qj_target_ = this->q_des_;
            this->com_start_ = this->com_ref_;
            this->com_target_ = this->com_ref_;
            this->transition_start_time_ = -std::numeric_limits<double>::infinity();
            this->v_des_.setZero();
            this->scheduled_contact_.assign(2, true);
            RCLCPP_INFO(logger, "walking stopped: standing at rest");
        }
        return true;
    }

    // ---------------------------------------------------------------------
    // Sizes the per-cycle buffers once (called from on_configure, after setupRobotModel and
    // joint_map_.build). update() only writes into them: an Eigen::VectorXd that is created or
    // resized allocates heap memory, which must not happen in the real-time loop.
    // ---------------------------------------------------------------------
    void WBCController::AllocateBuffers ()
    {
        const tsid::robots::RobotWrapper &robot = this->robotModel();
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

        // planned contact (contact fusion input): standing, both feet in stance mid-phase
        this->scheduled_contact_.assign(2, true);
        this->contact_phase_.assign(2, 0.5);

        // TSID solution
        this->dv_ = Eigen::VectorXd::Zero(nv);
        this->tau_ = Eigen::VectorXd::Zero(na);

        // joint command, Pinocchio order (robot_->write converts it to ros2_control order)
        this->q_cmd_ = Eigen::VectorXd::Zero(na);
        this->v_cmd_ = Eigen::VectorXd::Zero(na);
        this->tau_cmd_ = Eigen::VectorXd::Zero(na);

        // walking: integrated joint command, swing foot reference
        this->q_des_ = Eigen::VectorXd::Zero(na);
        this->v_des_ = Eigen::VectorXd::Zero(na);
        this->swing_pose_ = pinocchio::SE3::Identity();
        this->swing_vel_.setZero();
        this->swing_acc_.setZero();

        // kinematics of a motion command start (startTransition)
        this->data_ = pinocchio::Data(robot.model());
        this->left_foot_id_ = robot.model().getFrameId(this->params_.left_foot_frame);
    }

    // ---------------------------------------------------------------------
    // on_configure: publisher of ~/estimate (layout in the header). The realtime wrapper publishes
    // from its own thread; the message is sized here so update() never allocates.
    // ---------------------------------------------------------------------
    void WBCController::setupDebugPublisher ()
    {
        // created once (on_configure may run again after a cleanup)
        if (!this->rt_debug_pub_)
        {
            this->debug_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
                "~/estimate", rclcpp::SystemDefaultsQoS());
            this->rt_debug_pub_ =
                std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(this->debug_pub_);
        }

        // message with its final size; the label documents the layout in the message itself
        this->rt_debug_pub_->lock();
        auto &msg = this->rt_debug_pub_->msg_;
        msg.layout.dim.resize(1);
        msg.layout.dim[0].label = "p_base[3] v_base[3] contact_prob[2] f_z[2] sole_z[2] contact[2] accel_bias[3]";
        msg.layout.dim[0].size = kDebugSize;
        msg.layout.dim[0].stride = kDebugSize;
        msg.data.assign(kDebugSize, 0.0);
        this->rt_debug_pub_->unlock();
    }

    // ---------------------------------------------------------------------
    // update(), after estimate(): the estimate to ~/estimate at 50 Hz. Real-time: trylock does
    // not wait, and the data is written into the preallocated message.
    // ---------------------------------------------------------------------
    void WBCController::publishDebug ()
    {
        // every kDebugDecimation-th cycle
        if (++this->debug_counter_ < kDebugDecimation)
        {
            return;
        }
        this->debug_counter_ = 0;

        // publisher thread still busy with the last message: skip this one
        if (!this->rt_debug_pub_->trylock())
        {
            return;
        }

        // values in the order of the layout (header)
        auto &d = this->rt_debug_pub_->msg_.data;
        const Vector3d p = this->state_estimator_->getBasePosEst();
        const Vector3d v = this->state_estimator_->getBaseVelEst();
        const Vector3d bias = this->state_estimator_->getAccelBiasEst();
        for (int k = 0; k < 3; ++k)
        {
            d[k] = p(k);
            d[3 + k] = v(k);
            d[14 + k] = bias(k);
        }
        for (std::size_t i = 0; i < 2; ++i)
        {
            d[6 + i] = this->contact_probability_(i);
            d[8 + i] = this->contact_fusion_->getFootWrench(i)(2);
            d[10 + i] = this->contact_fusion_->getFootHeight(i);
            d[12 + i] = this->contact_fusion_->getContactState(i) ? 1.0 : 0.0;
        }
        this->rt_debug_pub_->unlockAndPublish();
    }

} // namespace biped_controllers

PLUGINLIB_EXPORT_CLASS(biped_controllers::WBCController, controller_interface::ControllerInterface)
