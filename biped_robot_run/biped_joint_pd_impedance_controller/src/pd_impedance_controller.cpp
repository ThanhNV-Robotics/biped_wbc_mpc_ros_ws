// Copyright (c) 2026, Van Thanh Nguyen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "biped_joint_pd_impedance_controller/pd_impedance_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace biped_joint_pd_impedance_controller
{

PdImpedanceController::PdImpedanceController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PdImpedanceController::on_init()
{
  try
  {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<bool>("verbose", true);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PdImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!load_parameters())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reserve space for initial positions and commands
  initial_positions_.resize(joint_names_.size(), 0.0);
  commanded_positions_.resize(joint_names_.size(), 0.0);
  commanded_velocities_.resize(joint_names_.size(), 0.0);
  commanded_efforts_.resize(joint_names_.size(), 0.0);

  // Create command subscriber
  command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "~/commands", 10,
      std::bind(&PdImpedanceController::command_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "PD Impedance Controller configured successfully");
  RCLCPP_INFO(get_node()->get_logger(), "Listening for commands on topic: ~/commands");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PdImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Clear command indices
  position_cmd_indices_.clear();
  velocity_cmd_indices_.clear();
  feedforward_effort_cmd_indices_.clear();
  position_state_indices_.clear();
  velocity_state_indices_.clear();
  effort_state_indices_.clear();

  // Map command and state interfaces to indices
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto& joint_name = joint_names_[i];

    // Find command interface indices
    auto pos_cmd_it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&joint_name](const auto& interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        });

    auto vel_cmd_it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&joint_name](const auto& interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });

    auto ff_cmd_it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&joint_name](const auto& interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == "feedforward_effort";
        });

    // Find state interface indices
    auto pos_state_it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&joint_name](const auto& interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        });

    auto vel_state_it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&joint_name](const auto& interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        });

    auto eff_state_it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&joint_name](const auto& interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
        });

    if (pos_cmd_it != command_interfaces_.end() && vel_cmd_it != command_interfaces_.end() &&
        ff_cmd_it != command_interfaces_.end() && pos_state_it != state_interfaces_.end() &&
        vel_state_it != state_interfaces_.end() && eff_state_it != state_interfaces_.end())
    {
      position_cmd_indices_.push_back(std::distance(command_interfaces_.begin(), pos_cmd_it));
      velocity_cmd_indices_.push_back(std::distance(command_interfaces_.begin(), vel_cmd_it));
      feedforward_effort_cmd_indices_.push_back(std::distance(command_interfaces_.begin(), ff_cmd_it));
      position_state_indices_.push_back(std::distance(state_interfaces_.begin(), pos_state_it));
      velocity_state_indices_.push_back(std::distance(state_interfaces_.begin(), vel_state_it));
      effort_state_indices_.push_back(std::distance(state_interfaces_.begin(), eff_state_it));
      
      // Store initial position
      initial_positions_[i] = state_interfaces_[std::distance(state_interfaces_.begin(), pos_state_it)].get_value();
      
      RCLCPP_INFO(get_node()->get_logger(), "Joint %s: initial position = %.3f", 
                  joint_name.c_str(), initial_positions_[i]);
    }
    else
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing interfaces for joint %s", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Initialize commanded values to current state
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    commanded_positions_[i] = initial_positions_[i];
    commanded_velocities_[i] = 0.0;
    commanded_efforts_[i] = 0.0;
  }
  
  initialized_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "PD Impedance Controller activated successfully");
  RCLCPP_INFO(get_node()->get_logger(), "Controller will hold current positions with zero velocity and feedforward");
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PdImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  initialized_ = false;
  RCLCPP_INFO(get_node()->get_logger(), "PD Impedance Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PdImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint : joint_names_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/feedforward_effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration PdImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint : joint_names_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

controller_interface::return_type PdImpedanceController::update(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/)
{
  if (!initialized_)
  {
    return controller_interface::return_type::OK;
  }

  // Update commands for all joints
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    double pos_cmd = has_command_ ? commanded_positions_[i] : initial_positions_[i];
    double vel_cmd = has_command_ ? commanded_velocities_[i] : 0.0;
    double ff_cmd = has_command_ ? commanded_efforts_[i] : 0.0;
    
    command_interfaces_[position_cmd_indices_[i]].set_value(pos_cmd);
    command_interfaces_[velocity_cmd_indices_[i]].set_value(vel_cmd);
    command_interfaces_[feedforward_effort_cmd_indices_[i]].set_value(ff_cmd);
  }
  
  // Debug logging
  if (verbose_ && has_command_)
  {
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Commanding %s to pos=%.3f, vel=%.3f, effort=%.3f",
                         joint_names_[0].c_str(), commanded_positions_[0], 
                         commanded_velocities_[0], commanded_efforts_[0]);
  }

  return controller_interface::return_type::OK;
}

bool PdImpedanceController::load_parameters()
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  verbose_ = get_node()->get_parameter("verbose").as_bool();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joints parameter is empty!");
    return false;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Loaded parameters:");
  RCLCPP_INFO(get_node()->get_logger(), "  - joints: %zu", joint_names_.size());
  for (const auto& joint : joint_names_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "    * %s", joint.c_str());
  }

  return true;
}

void PdImpedanceController::command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Store command in realtime buffer
  command_buffer_.writeFromNonRT(msg);
  
  // Update commanded values for each joint
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const auto& joint_name = joint_names_[i];
    
    // Find this joint in the message
    auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
    if (it != msg->name.end())
    {
      size_t idx = std::distance(msg->name.begin(), it);
      
      // Update position command if provided
      if (idx < msg->position.size())
      {
        commanded_positions_[i] = msg->position[idx];
      }
      
      // Update velocity command if provided
      if (idx < msg->velocity.size())
      {
        commanded_velocities_[i] = msg->velocity[idx];
      }
      
      // Update effort (feedforward) command if provided
      if (idx < msg->effort.size())
      {
        commanded_efforts_[i] = msg->effort[idx];
      }
    }
  }
  
  has_command_ = true;
  
  if (verbose_)
  {
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Received command with %zu joints", msg->name.size());
  }
}

}  // namespace biped_joint_pd_impedance_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(biped_joint_pd_impedance_controller::PdImpedanceController, controller_interface::ControllerInterface)
