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

  // declare ROS parameters (urdf_file, task_file, reference_file)
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  // ROS2 params instead of getParam()
  urdfFile      = get_node()->declare_parameter<std::string>("urdf_file", "");
  taskFile      = get_node()->declare_parameter<std::string>("task_file", "");
  referenceFile = get_node()->declare_parameter<std::string>("reference_file", "");

  if (urdfFile.empty() || taskFile.empty() || referenceFile.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "urdf_file, task_file, and reference_file parameters must be set");
    return controller_interface::CallbackReturn::FAILURE;
  }

  // print out a message that confirm the parameters are loaded successfully
  RCLCPP_INFO(get_node()->get_logger(), "URDF file: %s", urdfFile.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Task file: %s", taskFile.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Reference file: %s", referenceFile.c_str());
  
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

void LeggedController::updateStatesEstimatate(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // TODO(step3): update state estimation (e.g., using a complementary filter or a Kalman filter)
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose)
{
  // TODO(step3): set up state estimation (e.g., subscribe to IMU and joint state topics)
  // This function ready to be called in taskfile.info which include the Kalman filter parameters and initilize the robot state estimation

  // step 1: init the Kalman filter state estimation
  // The estimator gets its own rclcpp::Node (not the lifecycle node) so it can
  // publish odom and subscribe independently of the controller lifecycle state.
  auto estimatorNode = std::make_shared<rclcpp::Node>("legged_state_estimator");
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(estimatorNode,
                                                          legged_interface_->getPinocchioInterface(),
                                                          legged_interface_->getCentroidalModelInfo(),
                                                          *eeKinematicsPtr_);
  // step 2: load the Kalman filter parameters from the task file
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

}// namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerInterface)
