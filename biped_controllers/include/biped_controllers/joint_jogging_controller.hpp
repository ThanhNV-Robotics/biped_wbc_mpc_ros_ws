#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace biped_controllers
{

/// Jog the joints from slider values (joint_state_publisher_gui), slowly and smoothly.
///
/// Test controller for the hardware interface (read and write path), not for real use.
/// Writes position = filtered target, velocity = 0, effort = 0 for every joint in `joints`; the
/// motors track the position with their own kp / kd (PVT mode).
///
/// Safety:
///   - on activation every target starts at the measured position (nothing jumps)
///   - a joint follows its slider only after that slider has been moved (the GUI starts at 0)
///   - the target moves at most `max_velocity` rad/s, then passes a first-order low-pass filter
///     with time constant `filter_time_constant`
///   - while the hardware reports its motors disabled (state interface `enabled_interface`), the
///     target follows the measured position, so enabling the motors later never makes them jump
///     back to a stale pose (the controller can be active while the hardware is inactive)
///
/// Parameters: joints (required), slider_topic (/slider_joint_states), max_velocity (0.3 rad/s),
/// filter_time_constant (0.1 s), enabled_interface (BipedSystem/enabled; empty = not used).
class JointJoggingController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Handed from the subscription (non-real-time) to update() through a RealtimeBuffer
  struct SliderTargets
  {
    std::vector<double> position;   // latest slider value per joint (joints_ order)
    std::vector<uint8_t> follow;    // 1 once that joint's slider has been moved
  };

  void on_slider(const sensor_msgs::msg::JointState::SharedPtr msg);
  void reset_sliders();

  // Parameters
  std::vector<std::string> joints_;
  std::string slider_topic_;
  double max_velocity_{0.3};
  double filter_time_constant_{0.1};
  std::string enabled_interface_;

  // Subscription side (non-real-time), guarded by slider_mutex_ against on_activate
  std::mutex slider_mutex_;
  std::unordered_map<std::string, std::size_t> joint_index_;
  SliderTargets slider_;
  std::vector<double> slider_first_;
  std::vector<uint8_t> slider_seen_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr slider_sub_;
  realtime_tools::RealtimeBuffer<SliderTargets> slider_buffer_;

  // Real-time side
  std::vector<double> limited_;    // rate-limited target per joint
  std::vector<double> filtered_;   // low-pass filtered target = what is commanded
  std::vector<std::size_t> pos_cmd_, vel_cmd_, eff_cmd_, pos_state_;   // indices into the loans
  std::size_t enabled_state_{0};   // index of enabled_interface in the state loans (if used)
};

}  // namespace biped_controllers
