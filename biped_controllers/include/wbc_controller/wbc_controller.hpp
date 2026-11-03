#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.hpp"
#include "semantic_components/imu_sensor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "humanoid_mpc_msgs/msg/motion_command.hpp"
#include "humanoid_mpc_msgs/msg/walking_velocity_command.hpp"
#include "nav_msgs/msg/odometry.hpp"


#include "biped_wbc/state_estimation/contact_fusion.hpp"
#include "biped_wbc/state_estimation/StateEstimator.hpp"
#include <tsid/robots/robot-wrapper.hpp>
#include "biped_wbc/QP_tsid/QPtsid.hpp"
#include "biped_wbc/planner/WalkPlanner.hpp"

#include "wbc_controller/controller_params.hpp"
#include "wbc_controller/joint_map.hpp"
#include "wbc_controller/robot_interface.hpp"



namespace biped_controllers
{


class WBCController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Steps of the lifecycle, virtual so that a variant of the controller can replace one of them
  // (e.g. a "cheater" controller whose setupStateEstimate() / estimate() use the simulator's
  // ground truth, as LeggedCheaterController in legged_control).

  // on_configure, after setupRobotModel: QP_TSID and WalkPlanner from the config yaml, control
  // period dt_; false (error logged) if a file / value is invalid
  virtual bool setupWbc ();

  // on_configure, after setupWbc (needs the model and dt_): StateEstimator and ContactFusion;
  // false (error logged) if a frame is invalid
  virtual bool setupStateEstimate ();

  // on_activate, after the bumpless hold and a first reading (sensor_ is valid): start the
  // estimators and the planner from the current state
  virtual void resetModules ();

  // update(): sensor_ -> estimated state q_, v_ and contact_probability_ (for the next cycle)
  virtual void estimate ();

  // update(): estimated state -> joint command q_cmd_ / v_cmd_ / tau_cmd_ (Pinocchio order);
  // false = no command this cycle, the last one is held
  virtual bool computeCommand (const rclcpp::Time &time);

  // on_configure, first step after the parameters: loads the URDF as a free-flyer robot model,
  // shared by every module; false (error logged) if the URDF cannot be loaded
  bool setupRobotModel ();

  // on_configure: sizes the per-cycle buffers from the robot model
  void AllocateBuffers ();

  // on_activate, after resetModules: joint targets of commands 1 / 2 and the CoM of each target
  // pose relative to the left foot (IK once); every reference held at the current state.
  // false (error logged) if the IK does not converge
  bool setupPostureTargets ();

  // on_configure: subscription /humanoid/motion_command (keyboard / joystick, B7)
  void setupCommandSubscriber ();
  // on_configure, after AllocateBuffers: drive kp / kd of every joint from the ros2_control <joint>
  // params of robot_description (the gains the hardware / simulated drives use); missing -> 0 (warned)
  void loadDriveGains ();

  // update(), after estimate(): a new motion command starts its transition (startTransition);
  // ignored while a transition runs. Real-time.
  void handleMotionCommand (const rclcpp::Time &time);

  // quintic from the current references (bumpless) to the joint target and to the CoM of that pose,
  // placed on the left foot where it is now; starts at time. Real-time (no allocation).
  void startTransition (const Eigen::VectorXd &qj_target, const Eigen::Vector3d &com_in_foot,
                        const rclcpp::Time &time);

  // handleMotionCommand, first WALKING message at rest in the init walking pose: walk planner from
  // the estimated state, base upright, joint command integrated from the last one. Real-time
  // except the planner reset (FK, CoM: no allocation)
  void startWalking (const rclcpp::Time &time);

  // computeCommand while walking (walk_vx of the yaml, 0 = in place): planner -> contacts, CoM, swing foot
  // -> TSID; stop when the WALKING messages stop; back to standing at rest. false = QP failed
  bool computeWalkCommand (const rclcpp::Time &time);

  // on_configure, params_.use_ground_truth only: subscription of the simulator's ground truth
  void setupGroundTruthSubscriber ();

  // use_ground_truth: base pose and twist of q_ / v_ from the last ground truth message (joints
  // stay measured); false if none arrived yet. Real-time
  bool applyGroundTruth ();

  // on_configure: debug topic ~/estimate (see debug_pub_)
  void setupDebugPublisher ();

  // update(), after estimate(): every kDebugDecimation-th cycle, the estimate to ~/estimate;
  // skipped if the publisher thread is still busy (never blocks). Real-time.
  void publishDebug ();

  // robot model of all modules (valid after setupRobotModel)
  tsid::robots::RobotWrapper &robotModel () { return *this->robot_pin_wrapper_; }

  // control period of all modules [s] (LIPPlanner sampling_frequency), set by setupWbc
  double dt_ = 0.0;

  // Parameters (declared in on_init, read in on_configure; defaults in ControllerParams, the WBC
  // yaml default in on_init, overridden by controllers.yaml)
  ControllerParams params_; //struct type


  // Robot model (Pinocchio model of the URDF with a free-flyer base), created in on_configure.
  // Every module keeps a reference to it (each has its own pinocchio::Data), so it is declared
  // before them: members are destroyed in reverse order, the modules go first.
  std::unique_ptr<tsid::robots::RobotWrapper> robot_pin_wrapper_;

  // Modules, created in on_configure(): their constructors need files (yaml, URDF) that are only
  // known there, and pluginlib constructs the controller without arguments, so they cannot be
  // plain members.
  std::unique_ptr<QP_TSID> qp_tsid_controller_;      // tsid qp module
  std::unique_ptr<WalkPlanner> walk_planner_;       // walk planner module (LIP CoM + swing foot)
  std::unique_ptr<StateEstimator> state_estimator_;  // state estimator module
  std::unique_ptr<ContactFusion> contact_fusion_;    // contact fusion module

  // Joint order maps ros2_control (params_.joints) <-> Pinocchio (q[7:], v[6:], tau), built in
  // on_configure once the robot model exists
  JointMap joint_map_;

  // ros2_control interfaces (joints, IMU, motors enabled): claim, bind, read, write, hold. Created
  // in on_configure (needs the parameters and the joint map); works in Pinocchio order.
  std::unique_ptr<RobotInterface> robot_ros_interface_;

  // Per-cycle buffers, sized once in on_configure() (AllocateBuffers): update() runs in the
  // real-time loop and only writes into them, it never allocates memory.
  // Joint vectors are in Pinocchio order unless noted (see joint_map_).
  RobotSensor sensor_;                    // joint q / dq / tau + IMU of this cycle (estimator input)
  Eigen::VectorXd q_;                     // configuration [base pos 3, quat 4 (x y z w), joints] (nq)
  Eigen::VectorXd v_;                     // velocity [base twist 6 (base frame), joints] (nv)
  Eigen::VectorXd contact_probability_;   // [left, right] in [0, 1]: from the contact fusion of the
                                          // previous cycle, used by the state estimator
  std::vector<bool> scheduled_contact_;   // [left, right] planned stance (contact fusion input);
                                          // both feet until the planner runs
  std::vector<double> contact_phase_;     // [left, right] subphase in [0, 1] of that plan
  // use_time_base false: the contact estimator saw the swing foot off the ground in this step
  // (a touchdown counts only after it, not the contact it still reports right after the liftoff)
  bool liftoff_seen_ = false;
  // contact FSM of the swing foot (Bledt et al. ICRA 2018, Sec. VI, use_time_base false):
  // SWING follows the swing trajectory; EARLY: contact detected before the step ends, the foot is
  // held where it touched down (swing task) until the contact is confirmed; LATE: the step reached
  // Tswing without contact, the planner waits (phase 1) and the foot keeps descending until a
  // contact (or late_touchdown_timeout); CONTACT: the foot is in TSID contact (carries load) and
  // the planner was told (touchdown()), the step ends at Tswing or at once if it was late
  enum class SwingState { SWING, EARLY, LATE, CONTACT };
  SwingState swing_state_ = SwingState::SWING;
  double early_start_time_ = 0.0;         // [s] update time of the early contact (t0 of the paper)
  pinocchio::SE3 hold_pose_;              // EARLY: swing foot reference, the touchdown pose (world)
  int prev_swing_foot_ = -1;              // swing foot of the last cycle, -1 = none
  Eigen::VectorXd dv_;                    // accelerations solved by TSID (nv)
  Eigen::VectorXd tau_;                   // joint torques solved by TSID (na)
  Eigen::VectorXd q_cmd_;                 // joint command to send (robot_->write): position (na)
  Eigen::VectorXd v_cmd_;                 // velocity (na)
  Eigen::VectorXd tau_cmd_;               // feedforward torque (na)

  // Commands 1 (homing) and 2 (init walking): quintic joint trajectory from the current reference
  // to a target over kTransitionDuration, tracked by the TSID posture task, and a quintic CoM
  // reference to the CoM of the target pose (both feet in contact). Between commands the last
  // target is held.
  static constexpr double kStandBaseHeight = 0.84;    // [m] base above the ground, command 2
  static constexpr double kTransitionDuration = 2.0;  // [s]
  Eigen::VectorXd qj_home_;               // command 1 target: all joints 0 (na)
  Eigen::VectorXd qj_stand_;              // command 2 target: IK, base kStandBaseHeight high (na)
  Eigen::Vector3d com_home_in_foot_;      // CoM of each target pose in the left foot frame
  Eigen::Vector3d com_stand_in_foot_;
  Eigen::VectorXd qj_start_, qj_target_;  // transition running / last one (na)
  Eigen::VectorXd qj_ref_, dqj_ref_, ddqj_ref_;     // trajectory sample of this cycle (na)
  Eigen::VectorXd com_start_, com_target_;          // world (3)
  Eigen::VectorXd com_ref_, dcom_ref_, ddcom_ref_;  // sample of this cycle (3)
  // [s] update time of the last start; -inf = none yet (the targets are held)
  double transition_start_time_ = -std::numeric_limits<double>::infinity();
  pinocchio::Data data_;                  // FK of the left foot in startTransition (sized in on_configure)
  pinocchio::FrameIndex left_foot_id_ = 0;

  // Walking (command 3): stepping in place while the WALKING messages arrive (key held)
  enum class Mode { STAND, WALK };
  Mode mode_ = Mode::STAND;               // STAND: commands 1 / 2 (transitions); WALK: walk planner
  bool in_walking_pose_ = false;          // last transition was init walking (walking allowed)
  static constexpr double kWalkTimeout = 0.3;  // [s] no WALKING message -> stop
  static constexpr double kRestTolerance = 0.002; // [m] planned CoM - CP at rest -> standing again
  static constexpr double kMinTouchdownPhase = 0.7; // use_time_base false: earlier touchdowns ignored (tsid_examples touchdown_phase_min)
  double walk_cmd_time_ = 0.0;            // [s] update time of the last WALKING message
  bool stop_requested_ = false;           // the planner was asked to stop (this walk)
  Eigen::VectorXd q_des_, v_des_;         // joint command while walking: the measured joint state
                                          // integrated one cycle with the TSID accelerations (na)
  Eigen::VectorXd drive_kp_, drive_kd_;   // drive gains per joint, Pinocchio order (na), loadDriveGains
  pinocchio::SE3 swing_pose_;             // swing foot reference of this cycle (world)
  pinocchio::SE3 base_pose_ = pinocchio::SE3::Identity(); // torso orientation reference while walking (yaw)
  Eigen::Matrix<double, 6, 1> base_vel_ = Eigen::Matrix<double, 6, 1>::Zero(), base_acc_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> swing_vel_, swing_acc_; // [linear; angular]
  Eigen::Matrix<double, 6, 1> zero6_ = Eigen::Matrix<double, 6, 1>::Zero();

  // Ground truth (params_.use_ground_truth, simulation only): the subscriber thread writes the
  // last message, estimate() reads it
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<nav_msgs::msg::Odometry>> ground_truth_buffer_;

  // Motion command (/humanoid/motion_command): the subscriber thread writes, update() reads.
  // seq counts the messages so update() sees a repeated command as a new one.
  struct MotionCommandSample
  {
    uint8_t command = 0;
    uint64_t seq = 0;
  };
  rclcpp::Subscription<humanoid_mpc_msgs::msg::MotionCommand>::SharedPtr command_sub_;
  realtime_tools::RealtimeBuffer<MotionCommandSample> command_buffer_;
  uint64_t command_seq_ = 0;               // subscriber thread: messages received
  uint64_t last_command_seq_ = 0;          // update(): last message handled

  // Turn command (/humanoid/walking_velocity_command, angular_velocity_z in [-1, 1]): the
  // subscriber thread writes, update() reads; no message for kWalkTimeout -> no turning
  struct TurnCommandSample
  {
    double wz = 0.0;
    uint64_t seq = 0;
  };
  rclcpp::Subscription<humanoid_mpc_msgs::msg::WalkingVelocityCommand>::SharedPtr turn_sub_;
  realtime_tools::RealtimeBuffer<TurnCommandSample> turn_buffer_;
  uint64_t turn_seq_ = 0;                  // subscriber thread: messages received
  uint64_t last_turn_seq_ = 0;             // update(): last message handled
  double turn_cmd_ = 0.0;                  // last turn command in [-1, 1]
  double turn_cmd_time_ = -std::numeric_limits<double>::infinity(); // [s] update time of its message

  // Debug topic ~/estimate (std_msgs/Float64MultiArray, 50 Hz), for PlotJuggler. data layout
  // (world frame, [left, right] per foot):
  //   0-2   base position estimate               (ground truth /simulator/floating_base_state pose)
  //   3-5   base linear velocity estimate        (ground truth twist.linear, also world frame)
  //   6-7   contact probability (fused)
  //   8-9   foot normal force f_z [N] (momentum observer)
  //   10-11 sole height [m] (lowest sole corner)
  //   12-13 contact state (hysteresis), 0 / 1
  //   14-16 accelerometer bias estimate [m/s^2]
  // Message sized once in setupDebugPublisher: publishDebug only overwrites the values.
  static constexpr int kDebugDecimation = 10;  // 500 Hz control / 10 = 50 Hz
  static constexpr std::size_t kDebugSize = 31; // layout: label of the message (publishDebug)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_debug_pub_;
  int debug_counter_ = 0;
};

}  // namespace biped_controllers
