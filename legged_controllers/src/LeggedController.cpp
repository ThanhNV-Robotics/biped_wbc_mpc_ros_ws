//
// Created by qiayuan on 2022/6/24.
//

#include "legged_controllers/LeggedController.h"

namespace legged {

controller_interface::InterfaceConfiguration LeggedController::command_interface_configuration() const
{
  // TODO(step2): return INDIVIDUAL with joint position/velocity/effort interfaces
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration LeggedController::state_interface_configuration() const
{
  // TODO(step2): return INDIVIDUAL with joint state + IMU state interfaces
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn LeggedController::on_init()
{
  // TODO(step2): declare ROS parameters (urdf_file, task_file, reference_file)
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // TODO(step3): load LeggedInterface, setup MPC, MRT, state estimator, WBC
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // TODO(step3): initialize observation, wait for initial MPC policy, start MPC thread
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // TODO(step3): stop MPC thread, release interfaces
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeggedController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // TODO(step3): state estimation → MPC query → WBC → safety check → write commands
  return controller_interface::return_type::OK;
}

// ── LeggedCheaterController ─────────────────────────────────────────────────

controller_interface::CallbackReturn LeggedCheaterController::on_init()
{
  // TODO(step3): same as base but uses ground-truth state estimate topic
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerInterface)
