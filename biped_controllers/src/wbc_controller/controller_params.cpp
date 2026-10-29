#include "wbc_controller/controller_params.hpp"

#include <exception>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace biped_controllers
{

void ControllerParams::declare(rclcpp_lifecycle::LifecycleNode &node) const
{
  // declare once, with this object's value as default (same as ControllerInterface::auto_declare)
  const auto declare = [&node](const std::string &name, const auto &default_value) {
    if (!node.has_parameter(name))
    {
      node.declare_parameter(name, default_value);
    }
  };

  declare("urdf_package", this->urdf_package);
  declare("urdf_file", this->urdf_file);
  declare("config_package", this->config_package);
  declare("config_file", this->config_file);
  declare("joints", this->joints);
  declare("left_foot_frame", this->left_foot_frame);
  declare("right_foot_frame", this->right_foot_frame);
  declare("imu_name", this->imu_name);
  declare("enabled_interface", this->enabled_interface);
  declare("use_ground_truth", this->use_ground_truth);
  declare("ground_truth_topic", this->ground_truth_topic);
  declare("torque_mode", this->torque_mode);
}

bool ControllerParams::read(rclcpp_lifecycle::LifecycleNode &node)
{
  const auto logger = node.get_logger();

  // values in effect (yaml, defaults, or set while the controller was unconfigured)
  this->urdf_package = node.get_parameter("urdf_package").as_string();
  this->urdf_file = node.get_parameter("urdf_file").as_string();
  this->config_package = node.get_parameter("config_package").as_string();
  this->config_file = node.get_parameter("config_file").as_string();
  this->joints = node.get_parameter("joints").as_string_array();
  this->left_foot_frame = node.get_parameter("left_foot_frame").as_string();
  this->right_foot_frame = node.get_parameter("right_foot_frame").as_string();
  this->imu_name = node.get_parameter("imu_name").as_string();
  this->enabled_interface = node.get_parameter("enabled_interface").as_string();
  this->use_ground_truth = node.get_parameter("use_ground_truth").as_bool();
  this->ground_truth_topic = node.get_parameter("ground_truth_topic").as_string();
  this->torque_mode = node.get_parameter("torque_mode").as_bool();

  // values without a usable default
  if (this->joints.empty())
  {
    RCLCPP_ERROR(logger, "parameter 'joints' is empty: list the controlled joints in controllers.yaml");
    return false;
  }
  if (this->config_file.empty())
  {
    RCLCPP_ERROR(logger, "parameter 'config_file' is empty");
    return false;
  }

  // absolute paths: package share directory + relative path
  try
  {
    this->urdf_path = ament_index_cpp::get_package_share_directory(this->urdf_package) + "/" + this->urdf_file;
    this->config_path = ament_index_cpp::get_package_share_directory(this->config_package) + "/" + this->config_file;
  }
  catch (const std::exception &e)  // package not found
  {
    RCLCPP_ERROR(logger, "package not found: %s", e.what());
    return false;
  }

  // print the values in effect, to see whether the yaml or the defaults were used
  RCLCPP_INFO(logger, "URDF: %s", this->urdf_path.c_str());
  RCLCPP_INFO(logger, "Config: %s", this->config_path.c_str());
  RCLCPP_INFO(logger, "%zu joints, feet '%s' / '%s', IMU '%s', enabled interface '%s'", this->joints.size(),
              this->left_foot_frame.c_str(), this->right_foot_frame.c_str(), this->imu_name.c_str(),
              this->enabled_interface.c_str());
  if (this->torque_mode)
  {
    RCLCPP_WARN(logger, "torque_mode: only the effort commands are claimed, tau = TSID torque (drive kp = kd = 0)");
  }
  if (this->use_ground_truth)
  {
    RCLCPP_WARN(logger, "use_ground_truth: base state from '%s' (simulation only), not from the state estimator",
                this->ground_truth_topic.c_str());
  }
  return true;
}

}  // namespace biped_controllers
