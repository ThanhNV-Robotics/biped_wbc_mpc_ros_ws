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
#include "sensor_msgs/msg/joint_state.hpp"


#include "biped_wbc/state_estimation/contact_fusion.hpp"
#include "biped_wbc/state_estimation/StateEstimator.hpp"
#include <tsid/robots/robot-wrapper.hpp>

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

private:
  // Created in on_configure() (they need the robot model from the URDF): pluginlib constructs
  // the controller without arguments, so they cannot be plain members.
  // Order matters: robot_ first, the estimators reference its model (destroyed in reverse order).
  std::unique_ptr<tsid::robots::RobotWrapper> robot_;
  std::unique_ptr<StateEstimator> state_estimator_;
  std::unique_ptr<ContactFusion> contact_fusion_;

};

}  // namespace biped_controllers
