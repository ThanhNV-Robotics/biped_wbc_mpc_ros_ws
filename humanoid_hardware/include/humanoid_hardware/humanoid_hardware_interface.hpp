#ifndef HUMANOID_HARDWARE_INTERFACE_HPP_
#define HUMANOID_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "humanoid_hardware/udp_communication.h"
#include "rclcpp/rclcpp.hpp"

namespace humanoid_hardware
{
struct JointInfo
{
  std::string name;
  uint16_t motor_id;
  int rpi_id;
};

struct ImuData 
{
  double qx{0}, qy{0}, qz{0}, qw{1};
  double wx{0}, wy{0}, wz{0};
  double ax{0}, ay{0}, az{0};
};

class HumanoidHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<UDPCommunication> udp_rpi1_comm_;
  std::unique_ptr<UDPCommunication> udp_rpi2_comm_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_torques_;

  std::vector<JointInfo> joint_map_;
  std::vector<uint16_t> motor_ids_rpi1_;
  std::vector<uint16_t> motor_ids_rpi2_;

  std::string imu_sensor_name_; 
  bool has_imu_{false};
  ImuData imu_;
};
}  // namespace humanoid_hardware
#endif  // HUMANOID_HARDWARE_INTERFACE_HPP_