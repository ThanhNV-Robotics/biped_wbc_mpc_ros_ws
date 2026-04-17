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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace biped_joint_pd_impedance_controller
{

/**
 * @brief Simple PD Impedance controller for testing the custom interface
 * 
 * This controller claims position, velocity, and feedforward_effort command interfaces
 * to match the behavior of real motor drivers that operate in PD impedance mode.
 * 
 * For now, it just holds the current position with zero velocity and feedforward.
 * This can be extended later to integrate MPC or other high-level planners.
 */
class PdImpedanceController : public controller_interface::ControllerInterface
{
public:
  PdImpedanceController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Joint names to control
  std::vector<std::string> joint_names_;
  
  // Indices for command interfaces
  std::vector<size_t> position_cmd_indices_;
  std::vector<size_t> velocity_cmd_indices_;
  std::vector<size_t> feedforward_effort_cmd_indices_;
  
  // Indices for state interfaces
  std::vector<size_t> position_state_indices_;
  std::vector<size_t> velocity_state_indices_;
  std::vector<size_t> effort_state_indices_;
  
  // Initial positions to hold
  std::vector<double> initial_positions_;
  
  // Command storage (from subscription)
  std::vector<double> commanded_positions_;
  std::vector<double> commanded_velocities_;
  std::vector<double> commanded_efforts_;
  bool has_command_{false};
  
  // Subscription for joint commands
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>> command_buffer_;
  
  // Flags
  bool initialized_{false};
  bool verbose_{true};
  
  // Helper functions
  bool load_parameters();
  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace biped_joint_pd_impedance_controller
