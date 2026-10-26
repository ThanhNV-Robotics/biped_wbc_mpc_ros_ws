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
#include "semantic_components/imu_sensor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


#include "biped_wbc/state_estimation/contact_fusion.hpp"
#include "biped_wbc/state_estimation/StateEstimator.hpp"
#include <tsid/robots/robot-wrapper.hpp>
#include "biped_wbc/QP_tsid/QPtsid.hpp"
#include "biped_wbc/planner/LIPTaskPlanner.hpp"



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

  // creates QP_TSID, LIPTaskPlanner, StateEstimator and ContactFusion from the parameters;
  // false (error logged) if a file is missing or invalid -> on_configure() fails
  bool InitControlModules ();

  // checks the `joints` parameter against the robot model (12 names, each an actuated 1-DoF
  // joint, no duplicates) and builds the index maps ros2_control order <-> Pinocchio order;
  // false (error logged) if a check fails -> on_configure() fails
  bool BuildJointMap ();

  // sizes the per-cycle buffers from the robot model (called from on_configure, after the modules)
  void AllocateBuffers ();

  // update(): joint + IMU state interfaces -> sensor_ (Pinocchio order); false if a value is not
  // finite (the cycle keeps the last command)
  bool ReadSensors ();

  // update(): q_cmd_ / v_cmd_ / tau_cmd_ -> command interfaces (ros2_control order)
  void WriteCommands ();

private:
  // Parameters (declared in on_init, defaults there, overridden by controllers.yaml)
  std::string urdf_package_;              // package of the URDF
  std::string urdf_file_;                 // URDF path relative to the package share directory
  std::string wbc_config_package_;        // package of the WBC yaml (TSID tasks, contacts, LIP planner)
  std::string wbc_controller_config_path; // WBC yaml path relative to that package's share directory
  std::vector<std::string> joint_names_;  // the 12 controlled joints, ros2_control names
  std::string left_foot_frame_;           // URDF frames of the feet (estimator, contact fusion)
  std::string right_foot_frame_;
  std::string imu_name_;                  // ros2_control sensor name: interfaces <imu_name>/orientation.x ...
  std::string enabled_interface_;         // hardware state interface, 1 = motors enabled


  // Modules, created in on_configure(): their constructors need files (yaml, URDF) that are only
  // known there, and pluginlib constructs the controller without arguments, so they cannot be
  // plain members. Order matters: qp_tsid_controller_ owns the robot model that the estimators
  // reference, so it is declared first (members are destroyed in reverse order).
  std::unique_ptr<QP_TSID> qp_tsid_controller_;      // tsid qp module (owns the robot wrapper)
  std::unique_ptr<LIPTaskPlanner> lip_task_planner_; // LIP task planner module
  std::unique_ptr<StateEstimator> state_estimator_;  // state estimator module
  std::unique_ptr<ContactFusion> contact_fusion_;    // contact fusion module

  // Joint order maps (built in on_configure by BuildJointMap). "ros2_control order" = order of
  // joint_names_ (the `joints` parameter, used for the interfaces); "Pinocchio order" = order of
  // the actuated joints in the model = order of q[7:], v[6:], tau of TSID and the estimators.
  //   ros_to_pin_[i] = Pinocchio index of joint_names_[i]
  //   pin_to_ros_[j] = index in joint_names_ of the j-th actuated joint of the model
  std::vector<int> ros_to_pin_;
  std::vector<int> pin_to_ros_;

  // IMU: ros2_control helper for the 10 state interfaces <imu_name>/orientation.x ...
  // /linear_acceleration.z. Created in on_configure (its constructor needs imu_name_), claimed in
  // state_interface_configuration(), bound in on_activate(), released in on_deactivate().
  std::unique_ptr<semantic_components::IMUSensor> imu_;

  // Joint interfaces: index of each joint's loaned interface in command_interfaces_ /
  // state_interfaces_, in ros2_control order (joint_names_[i]). Found by name in on_activate (the
  // loan order is not guaranteed), then used every cycle without searching.
  std::vector<std::size_t> pos_cmd_idx_, vel_cmd_idx_, eff_cmd_idx_;       // command interfaces
  std::vector<std::size_t> pos_state_idx_, vel_state_idx_, eff_state_idx_; // state interfaces
  std::size_t enabled_state_idx_{0};  // index of enabled_interface_ (1 = motors enabled)

  // Per-cycle buffers, sized once in on_configure() (AllocateBuffers): update() runs in the
  // real-time loop and only writes into them, it never allocates memory.
  // Joint vectors are in Pinocchio order unless noted (see ros_to_pin_ / pin_to_ros_).
  RobotSensor sensor_;                    // joint q / dq / tau + IMU of this cycle (estimator input)
  Eigen::VectorXd q_;                     // configuration [base pos 3, quat 4 (x y z w), joints] (nq)
  Eigen::VectorXd v_;                     // velocity [base twist 6 (base frame), joints] (nv)
  Eigen::VectorXd contact_probability_;   // [left, right] in [0, 1]: from the contact fusion of the
                                          // previous cycle, used by the state estimator
  Eigen::VectorXd dv_;                    // accelerations solved by TSID (nv)
  Eigen::VectorXd tau_;                   // joint torques solved by TSID (na)
  Eigen::VectorXd q_cmd_;                 // commands written to the interfaces, ros2_control order (na)
  Eigen::VectorXd v_cmd_;
  Eigen::VectorXd tau_cmd_;
};

}  // namespace biped_controllers
