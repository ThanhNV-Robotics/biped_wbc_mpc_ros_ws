#include "biped_controllers/joint_jogging_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace biped_controllers
{

namespace
{
constexpr double kSliderMovedRad = 1e-3;   // a slider counts as moved once it leaves its first value
constexpr double kMaxDtS = 0.02;           // ignore longer periods (e.g. after a stall) in the filter
}  // namespace

controller_interface::CallbackReturn JointJoggingController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::string>("slider_topic", "/slider_joint_states");
    auto_declare<double>("max_velocity", 0.3);
    auto_declare<double>("filter_time_constant", 0.1);
    auto_declare<std::string>("enabled_interface", "BipedSystem/enabled");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Declaring parameters failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointJoggingController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointJoggingController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  if (!enabled_interface_.empty()) {
    config.names.push_back(enabled_interface_);
  }
  return config;
}

controller_interface::CallbackReturn JointJoggingController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  slider_topic_ = node->get_parameter("slider_topic").as_string();
  max_velocity_ = node->get_parameter("max_velocity").as_double();
  filter_time_constant_ = node->get_parameter("filter_time_constant").as_double();
  enabled_interface_ = node->get_parameter("enabled_interface").as_string();

  if (joints_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!(max_velocity_ > 0.0) || !(filter_time_constant_ >= 0.0)) {
    RCLCPP_ERROR(node->get_logger(), "max_velocity must be > 0 and filter_time_constant >= 0");
    return controller_interface::CallbackReturn::ERROR;
  }

  const std::size_t n = joints_.size();
  joint_index_.clear();
  for (std::size_t i = 0; i < n; ++i) {
    joint_index_[joints_[i]] = i;
  }
  slider_.position.assign(n, 0.0);
  slider_.follow.assign(n, 0);
  slider_first_.assign(n, 0.0);
  slider_seen_.assign(n, 0);
  limited_.assign(n, 0.0);
  filtered_.assign(n, 0.0);
  slider_buffer_.writeFromNonRT(slider_);

  slider_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    slider_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&JointJoggingController::on_slider, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(), "%zu joints, sliders on %s, max %.2f rad/s, filter %.2f s",
    n, slider_topic_.c_str(), max_velocity_, filter_time_constant_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointJoggingController::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Map each joint to its loaned interfaces by name (don't rely on the loan order)
  const std::size_t n = joints_.size();
  const auto find = [](const auto & loans, const std::string & joint, const char * iface) {
      for (std::size_t k = 0; k < loans.size(); ++k) {
        if (loans[k].get_prefix_name() == joint && loans[k].get_interface_name() == iface) {
          return k;
        }
      }
      return loans.size();
    };
  pos_cmd_.assign(n, 0);
  vel_cmd_.assign(n, 0);
  eff_cmd_.assign(n, 0);
  pos_state_.assign(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    pos_cmd_[i] = find(command_interfaces_, joints_[i], hardware_interface::HW_IF_POSITION);
    vel_cmd_[i] = find(command_interfaces_, joints_[i], hardware_interface::HW_IF_VELOCITY);
    eff_cmd_[i] = find(command_interfaces_, joints_[i], hardware_interface::HW_IF_EFFORT);
    pos_state_[i] = find(state_interfaces_, joints_[i], hardware_interface::HW_IF_POSITION);
    if (pos_cmd_[i] == command_interfaces_.size() || vel_cmd_[i] == command_interfaces_.size() ||
      eff_cmd_[i] == command_interfaces_.size() || pos_state_[i] == state_interfaces_.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing interfaces for joint '%s'", joints_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  enabled_state_ = state_interfaces_.size();
  if (!enabled_interface_.empty()) {
    for (std::size_t k = 0; k < state_interfaces_.size(); ++k) {
      if (state_interfaces_[k].get_name() == enabled_interface_) {enabled_state_ = k;}
    }
    if (enabled_state_ == state_interfaces_.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface '%s'", enabled_interface_.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Start exactly where the robot is, hold it until a slider moves
  for (std::size_t i = 0; i < n; ++i) {
    const double q = state_interfaces_[pos_state_[i]].get_value();
    if (!std::isfinite(q)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' has no valid position", joints_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    limited_[i] = q;
    filtered_[i] = q;
    command_interfaces_[pos_cmd_[i]].set_value(q);
    command_interfaces_[vel_cmd_[i]].set_value(0.0);
    command_interfaces_[eff_cmd_[i]].set_value(0.0);
  }
  reset_sliders();
  RCLCPP_INFO(get_node()->get_logger(), "Active: holding the current pose, move a slider to jog its joint");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointJoggingController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // The hardware keeps the last commands (hold the last target)
  return controller_interface::CallbackReturn::SUCCESS;
}

void JointJoggingController::reset_sliders()
{
  std::lock_guard<std::mutex> lock(slider_mutex_);
  std::fill(slider_.follow.begin(), slider_.follow.end(), 0);
  std::fill(slider_seen_.begin(), slider_seen_.end(), 0);
  slider_buffer_.writeFromNonRT(slider_);
}

void JointJoggingController::on_slider(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(slider_mutex_);
  const std::size_t count = std::min(msg->name.size(), msg->position.size());
  for (std::size_t k = 0; k < count; ++k) {
    const auto it = joint_index_.find(msg->name[k]);
    if (it == joint_index_.end() || !std::isfinite(msg->position[k])) {
      continue;
    }
    const std::size_t i = it->second;
    const double value = msg->position[k];
    if (!slider_seen_[i]) {
      slider_seen_[i] = 1;
      slider_first_[i] = value;
    }
    slider_.position[i] = value;
    if (!slider_.follow[i] && std::fabs(value - slider_first_[i]) > kSliderMovedRad) {
      slider_.follow[i] = 1;
      RCLCPP_INFO(get_node()->get_logger(), "%s now follows its slider", joints_[i].c_str());
    }
  }
  slider_buffer_.writeFromNonRT(slider_);
}

controller_interface::return_type JointJoggingController::update(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const SliderTargets * sliders = slider_buffer_.readFromRT();
  const double dt = std::clamp(period.seconds(), 0.0, kMaxDtS);
  const double step_max = max_velocity_ * dt;
  const double alpha = filter_time_constant_ > 0.0 ? dt / (filter_time_constant_ + dt) : 1.0;

  // Motors disabled: track the measured pose, so enabling never jumps to a stale target
  const bool motors_enabled = enabled_state_ >= state_interfaces_.size() ||
    state_interfaces_[enabled_state_].get_value() > 0.5;

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (!motors_enabled) {
      const double q = state_interfaces_[pos_state_[i]].get_value();
      if (std::isfinite(q)) {
        limited_[i] = q;
        filtered_[i] = q;
      }
      command_interfaces_[pos_cmd_[i]].set_value(filtered_[i]);
      command_interfaces_[vel_cmd_[i]].set_value(0.0);
      command_interfaces_[eff_cmd_[i]].set_value(0.0);
      continue;
    }
    const bool follow = sliders != nullptr && sliders->follow[i];
    const double goal = follow ? sliders->position[i] : limited_[i];   // else hold
    limited_[i] += std::clamp(goal - limited_[i], -step_max, step_max);
    filtered_[i] += alpha * (limited_[i] - filtered_[i]);

    command_interfaces_[pos_cmd_[i]].set_value(filtered_[i]);
    command_interfaces_[vel_cmd_[i]].set_value(0.0);
    command_interfaces_[eff_cmd_[i]].set_value(0.0);
  }
  return controller_interface::return_type::OK;
}

}  // namespace biped_controllers

PLUGINLIB_EXPORT_CLASS(biped_controllers::JointJoggingController, controller_interface::ControllerInterface)
