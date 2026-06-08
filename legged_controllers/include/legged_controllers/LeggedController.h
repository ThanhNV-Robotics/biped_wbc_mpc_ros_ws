//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace legged {

class LeggedController : public controller_interface::ControllerInterface
{
public:
  LeggedController() = default;
  ~LeggedController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
};

class LeggedCheaterController : public LeggedController
{
public:
  LeggedCheaterController() = default;
  ~LeggedCheaterController() override = default;

  controller_interface::CallbackReturn on_init() override;
};

}  // namespace legged
