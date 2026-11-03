#pragma once

#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace biped_controllers
{

// Parameters every biped controller has (robot model, joints, feet, IMU, motor state, its own
// config yaml). Declared in on_init(), read in on_configure().
//
// The member values below are the defaults: a controller changes the ones that differ for it
// (e.g. config_file) before calling declare(); a value in controllers.yaml overrides them.
struct ControllerParams
{
  // robot model: package + URDF path inside its share directory
  std::string urdf_package = "biped_description";
  std::string urdf_file = "urdf/v2_biped_robot_12dof.urdf";
  // controller config yaml (TSID tasks, contacts, planner ...): package + path inside its share directory
  std::string config_package = "biped_bringup";
  std::string config_file;                       // no default: set by the controller
  // the controlled joints, ros2_control names; no default, must come from controllers.yaml
  std::vector<std::string> joints;
  // URDF frames of the feet (state estimator, contact fusion, TSID contacts)
  std::string left_foot_frame = "left_ankle_pitch_link";
  std::string right_foot_frame = "right_ankle_pitch_link";
  // ros2_control sensor name of the IMU: interfaces <imu_name>/orientation.x, ...
  std::string imu_name = "imu";
  // hardware state interface: 1 = motors enabled, 0 = off
  std::string enabled_interface = "BipedSystem/enabled";
  // simulation only ("cheat" estimator, as Cheat_StateEstimator of tsid_examples): base pose and
  // twist from the simulator's ground truth (nav_msgs/Odometry) instead of the state estimator
  bool use_ground_truth = false;
  std::string ground_truth_topic = "/simulator/floating_base_state";
  // pure torque control: claim only the joint effort commands and send the TSID torque (the drive
  // runs with kp = kd = 0: mujoco_ros2_control torque mode, it holds the pose with its gains until
  // the claim). false: PVT command (position, velocity, feedforward torque) with the drive gains
  bool torque_mode = false;
  // [m/s] forward walking speed of the walk command (step length = walk_vx * Tswing, clamped to
  // [0, max_step_length] of the planner yaml); 0 = stepping in place
  double walk_vx = 0.0;
  // [rad/s] yaw rate at full turn command: angular_velocity_z in [-1, 1] of
  // /humanoid/walking_velocity_command times this (left turn positive); 0 = no turning
  double max_yaw_rate = 0.2;
  // contact switching while walking. true: time based, the swing foot lands at the end of the
  // step (Tswing, phase variable of the planner). false: event based (contact FSM of Bledt et al.
  // ICRA 2018, Sec. VI): when the contact estimator (contact fusion) detects the swing foot on the
  // ground, from step phase 0.5 on, the foot is held where it is (early touchdown) and lands in
  // TSID once the contact has held for touchdown_confirm_time; without contact at Tswing the step
  // waits (late touchdown, the foot keeps descending) until the contact. The liftoff stays time based
  bool use_time_base = true;
  // [s] use_time_base false: an early contact must hold this long before the foot carries load
  // (the paper's t_delay), so a fleeting touch or a slip does not land the foot
  double touchdown_confirm_time = 0.025;
  // [s] use_time_base false: longest wait for a late touchdown (no contact at the end of the step);
  // then the foot lands in TSID anyway (in the air). 0 = no timeout, wait for the contact
  double late_touchdown_timeout = 0.0;

  // resolved by read(): absolute paths of the URDF and of the config yaml
  std::string urdf_path;
  std::string config_path;

  // on_init(): declares every parameter with the values of this object as defaults (a parameter
  // already declared is kept). Throws on a yaml value of the wrong type.
  void declare(rclcpp_lifecycle::LifecycleNode &node) const;

  // on_configure(): reads the values in effect (a `ros2 param set` after on_init counts), checks
  // them and resolves urdf_path / config_path. false (error logged) if a value is invalid or a
  // package is not found.
  bool read(rclcpp_lifecycle::LifecycleNode &node);
};

}  // namespace biped_controllers
