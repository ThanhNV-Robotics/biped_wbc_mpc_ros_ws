#pragma once

#include <hardware_interface/version.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#define LEGGED_HW_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

#if !LEGGED_HW_HUMBLE
#include <hardware_interface/types/hardware_component_params.hpp>
#endif

#include "legged_hw/udp_bridge.hpp"

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace legged_hw {

class LeggedHW : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn
#if LEGGED_HW_HUMBLE
  on_init(const hardware_interface::HardwareInfo& info) override;
#else
  on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
#endif

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void recvLoop();

  // Per-joint state + command buffers (indexed by joint order in URDF)
  std::vector<std::string> joint_names_;
  std::size_t n_joints_{0};

  std::vector<double> pos_state_;
  std::vector<double> vel_state_;
  std::vector<double> eff_state_;

  std::vector<double> pos_cmd_;
  std::vector<double> vel_cmd_;
  std::vector<double> eff_cmd_;
  std::vector<double> kp_cmd_;
  std::vector<double> kd_cmd_;

  // IMU state — sensor name read from URDF (default "imu_sensor")
  std::string imu_sensor_name_{"imu_sensor"};
  std::array<double, 4> imu_orientation_{{1.0, 0.0, 0.0, 0.0}};  // w, x, y, z
  std::array<double, 3> imu_angular_vel_{{}};
  std::array<double, 3> imu_linear_acc_{{}};

  // UDP receive (state from sim/robot) and send (commands to sim/robot)
  UdpSocket recv_sock_;
  UdpSocket send_sock_;

  std::thread recv_thread_;
  std::atomic<bool> recv_running_{false};

  RobotStatePacket latest_state_{};
  std::mutex       state_mutex_;
  bool             state_fresh_{false};

  std::string sim_ip_{"127.0.0.1"};
  uint16_t    state_port_{9870};
  uint16_t    cmd_port_{9871};
};

}  // namespace legged_hw
