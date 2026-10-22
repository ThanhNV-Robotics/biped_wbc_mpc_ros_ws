#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "biped_hardware/udp_client.hpp"
#include "biped_hardware/wbc_protocol.h"

namespace biped_hardware
{

/// ros2_control hardware interface for the 12-motor biped behind the NUCLEO-H755 bridge.
///
/// One WbcCommandPacket per write(), the reply is picked up by the next read().
/// Joint space (URDF) <-> motor space (wbc_protocol.h motors[] index):
///   - hips / knees: joint = direction * motor
///   - ankles: the roll joint carries the upper motor (50 / 110), the pitch joint the lower
///     motor (60 / 120); ankle_kinematics.hpp maps the motor pair to roll / pitch.
///
/// Step 2 (read path): motors are never enabled, write() only sends ENABLE = 0 packets
/// so the board keeps answering.
class BipedSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct JointConfig
  {
    std::string name;
    int motor_index{-1};   // index into WbcCommandPacket / WbcStatePacket motors[]
    double direction{1.0};  // +1 / -1, applied in motor space
  };

  struct AnklePair
  {
    int roll{-1};   // joint index of *_ankle_roll_joint (upper motor)
    int pitch{-1};  // joint index of *_ankle_pitch_joint (lower motor)
  };

  void send_disabled(int count);
  void motor_state_to_joints(const WbcStatePacket & st);
  void report_status(const WbcStatePacket & st);

  rclcpp::Logger logger_{rclcpp::get_logger("BipedSystem")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // Configuration (hardware parameters in the URDF <ros2_control> block)
  std::string board_ip_{"192.168.10.10"};
  uint16_t board_port_{WBC_UDP_PORT};
  int timeout_cycles_{10};  // read() errors after this many cycles without a reply

  std::vector<JointConfig> joints_;
  AnklePair ankle_left_;
  AnklePair ankle_right_;

  // State feedback, joint space
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;

  // Commands, joint space (not sent yet in step 2)
  std::vector<double> position_command_;
  std::vector<double> velocity_command_;
  std::vector<double> effort_command_;
  std::vector<double> kp_command_;
  std::vector<double> kd_command_;

  // Link to the board
  UdpClient udp_;
  WbcCommandPacket cmd_{};
  WbcStatePacket state_{};
  uint32_t seq_{0};
  int missed_cycles_{0};
  bool cm4_data_{true};  // last seen !NO_CM4_DATA; starts true so placeholders are reported
};

}  // namespace biped_hardware
