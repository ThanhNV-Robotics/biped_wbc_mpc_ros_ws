#pragma once

#include <cstdint>
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


#include "biped_wbc/state_estimation/contact_fusion.hpp"
#include "biped_wbc/state_estimation/StateEstimator.hpp"
#include <tsid/robots/robot-wrapper.hpp>
#include "biped_wbc/QP_tsid/QPtsid.hpp"
#include "biped_wbc/planner/LIPTaskPlanner.hpp"

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

  // on_configure, after setupRobotModel: QP_TSID and LIPTaskPlanner from the config yaml, control
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

  // on_activate, after resetModules: stand-down test (command 2, B8). Joint target from the IK
  // (once): feet where they are relative to the base, base kStandBaseHeight above the ground.
  // false (error logged) if the IK does not converge
  bool setupStandDown ();

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
  std::unique_ptr<LIPTaskPlanner> lip_task_planner_; // LIP task planner module
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
  Eigen::VectorXd dv_;                    // accelerations solved by TSID (nv)
  Eigen::VectorXd tau_;                   // joint torques solved by TSID (na)
  Eigen::VectorXd q_cmd_;                 // joint command to send (robot_->write): position (na)
  Eigen::VectorXd v_cmd_;                 // velocity (na)
  Eigen::VectorXd tau_cmd_;               // feedforward torque (na)

  // Stand-down test (command 2, B8): quintic joint trajectory from the pose at activation to the
  // IK target over kStandDuration, tracked by the TSID posture task (both feet in contact)
  static constexpr double kStandBaseHeight = 0.78;  // [m] base above the ground at the end
  static constexpr double kStandDuration = 2.0;     // [s]
  Eigen::VectorXd qj_start_;              // joints at activation (na)
  Eigen::VectorXd qj_target_;             // IK target (na)
  Eigen::VectorXd qj_ref_, dqj_ref_, ddqj_ref_;  // trajectory sample of this cycle (na)
  // CoM: quintic from the CoM at activation to the CoM of the target pose (TSID CoM task), world
  Eigen::VectorXd com_start_, com_target_;       // (3)
  Eigen::VectorXd com_ref_, dcom_ref_, ddcom_ref_;  // sample of this cycle (3)
  rclcpp::Time stand_start_time_;         // time of the first update() after activation
  bool stand_started_ = false;

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
  static constexpr std::size_t kDebugSize = 17;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_debug_pub_;
  int debug_counter_ = 0;
};

}  // namespace biped_controllers
