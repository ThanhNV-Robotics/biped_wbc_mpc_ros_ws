#include "legged_hw/legged_hw.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <chrono>
#include <cstring>
#include <stdexcept>

namespace legged_hw {

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static std::string hw_param(const hardware_interface::HardwareInfo& info,
                             const std::string& key,
                             const std::string& def = "")
{
  const auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end()) ? it->second : def;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_init
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
#if LEGGED_HW_HUMBLE
LeggedHW::on_init(const hardware_interface::HardwareInfo& info)
#else
LeggedHW::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
#endif
{
#if LEGGED_HW_HUMBLE
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
#else
  if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
#endif
    return hardware_interface::CallbackReturn::ERROR;

  const hardware_interface::HardwareInfo& hw_info = get_hardware_info();

  // UDP config — use_sim selects MuJoCo (sim_ip) or real hardware (hw_ip)
  use_sim_ = hw_param(hw_info, "use_sim", "true") == "true";
  const std::string sim_ip = hw_param(hw_info, "sim_ip", UDP_DEFAULT_SIM_IP);
  const std::string hw_ip  = hw_param(hw_info, "hw_ip",  UDP_DEFAULT_HW_IP);
  target_ip_  = use_sim_ ? sim_ip : hw_ip;
  state_port_ = static_cast<uint16_t>(std::stoi(hw_param(hw_info, "state_port",
                                                 std::to_string(UDP_DEFAULT_STATE_PORT))));
  cmd_port_   = static_cast<uint16_t>(std::stoi(hw_param(hw_info, "cmd_port",
                                                 std::to_string(UDP_DEFAULT_CMD_PORT))));

  // IMU sensor name
  if (!hw_info.sensors.empty())
    imu_sensor_name_ = hw_info.sensors[0].name;

  // Build joint name list from URDF order
  n_joints_ = hw_info.joints.size();
  joint_names_.resize(n_joints_);
  for (std::size_t i = 0; i < n_joints_; ++i)
    joint_names_[i] = hw_info.joints[i].name;

  pos_state_.assign(n_joints_, 0.0);
  vel_state_.assign(n_joints_, 0.0);
  eff_state_.assign(n_joints_, 0.0);
  pos_cmd_.assign(n_joints_, 0.0);
  vel_cmd_.assign(n_joints_, 0.0);
  eff_cmd_.assign(n_joints_, 0.0);
  kp_cmd_.assign(n_joints_, 0.0);
  kd_cmd_.assign(n_joints_, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("LeggedHW"),
              "Initialized %zu joints — mode:%s  target:%s  state_port:%u  cmd_port:%u",
              n_joints_, use_sim_ ? "SIM" : "REAL_HW",
              target_ip_.c_str(), state_port_, cmd_port_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces
// ─────────────────────────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface> LeggedHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  ifaces.reserve(n_joints_ * 3 + 10);

  for (std::size_t i = 0; i < n_joints_; ++i) {
    ifaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_state_[i]);
    ifaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_state_[i]);
    ifaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT,   &eff_state_[i]);
  }

  // IMU — imu_sensor_broadcaster expects these exact interface names
  ifaces.emplace_back(imu_sensor_name_, "orientation.w",            &imu_orientation_[0]);
  ifaces.emplace_back(imu_sensor_name_, "orientation.x",            &imu_orientation_[1]);
  ifaces.emplace_back(imu_sensor_name_, "orientation.y",            &imu_orientation_[2]);
  ifaces.emplace_back(imu_sensor_name_, "orientation.z",            &imu_orientation_[3]);
  ifaces.emplace_back(imu_sensor_name_, "angular_velocity.x",       &imu_angular_vel_[0]);
  ifaces.emplace_back(imu_sensor_name_, "angular_velocity.y",       &imu_angular_vel_[1]);
  ifaces.emplace_back(imu_sensor_name_, "angular_velocity.z",       &imu_angular_vel_[2]);
  ifaces.emplace_back(imu_sensor_name_, "linear_acceleration.x",    &imu_linear_acc_[0]);
  ifaces.emplace_back(imu_sensor_name_, "linear_acceleration.y",    &imu_linear_acc_[1]);
  ifaces.emplace_back(imu_sensor_name_, "linear_acceleration.z",    &imu_linear_acc_[2]);

  return ifaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_command_interfaces
// ─────────────────────────────────────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface> LeggedHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  ifaces.reserve(n_joints_ * 5);

  for (std::size_t i = 0; i < n_joints_; ++i) {
    ifaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_cmd_[i]);
    ifaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_cmd_[i]);
    ifaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT,   &eff_cmd_[i]);
    ifaces.emplace_back(joint_names_[i], "kp",                               &kp_cmd_[i]);
    ifaces.emplace_back(joint_names_[i], "kd",                               &kd_cmd_[i]);
  }

  return ifaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate / on_deactivate
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
LeggedHW::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Open UDP receive socket (state FROM sim/robot)
  if (!recv_sock_.open_receiver(state_port_)) {
    RCLCPP_ERROR(rclcpp::get_logger("LeggedHW"),
                 "Failed to bind UDP receive socket on port %u", state_port_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Open UDP send socket (commands TO sim/robot)
  if (!send_sock_.open_sender(target_ip_, cmd_port_)) {
    RCLCPP_ERROR(rclcpp::get_logger("LeggedHW"),
                 "Failed to open UDP send socket to %s:%u", target_ip_.c_str(), cmd_port_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  recv_running_ = true;
  recv_thread_  = std::thread(&LeggedHW::recvLoop, this);
  watchdog_thread_ = std::thread(&LeggedHW::watchdogLoop, this);

  RCLCPP_INFO(rclcpp::get_logger("LeggedHW"),
              "LeggedHW activated — listening on :%u, sending to %s:%u",
              state_port_, target_ip_.c_str(), cmd_port_);
  RCLCPP_INFO(rclcpp::get_logger("LeggedHW"),
              "Waiting for hardware connection (UDP packets on port %u)...", state_port_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LeggedHW::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  recv_running_ = false;
  recv_sock_.close();  // unblocks recvfrom
  if (recv_thread_.joinable())    recv_thread_.join();
  if (watchdog_thread_.joinable()) watchdog_thread_.join();
  send_sock_.close();

  RCLCPP_INFO(rclcpp::get_logger("LeggedHW"), "LeggedHW deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// recvLoop — background thread
// ─────────────────────────────────────────────────────────────────────────────

void LeggedHW::recvLoop()
{
  RobotStatePacket pkt{};
  while (recv_running_) {
    const ssize_t n = recv_sock_.recv(&pkt, sizeof(pkt));
    if (n != static_cast<ssize_t>(sizeof(pkt))) continue;  // timeout or bad size

    std::lock_guard<std::mutex> lk(state_mutex_);
    latest_state_ = pkt;
    state_fresh_  = true;
    if (!hw_connected_) {
      hw_connected_ = true;
      RCLCPP_INFO(rclcpp::get_logger("LeggedHW"), "Hardware connected — receiving UDP data.");
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// watchdogLoop — logs a warning every 2 s while no hardware data arrives
// ─────────────────────────────────────────────────────────────────────────────

void LeggedHW::watchdogLoop()
{
  constexpr int WARN_INTERVAL_S = 2;
  while (recv_running_) {
    for (int i = 0; i < WARN_INTERVAL_S * 10 && recv_running_; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!recv_running_) break;

    bool connected = false;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      connected = hw_connected_;
    }
    if (!connected)
      RCLCPP_WARN(rclcpp::get_logger("LeggedHW"),
                  "No UDP data from hardware yet — waiting for %s:%u ...",
                  target_ip_.c_str(), state_port_);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// read — copy latest UDP state into ros2_control state interfaces
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::return_type
LeggedHW::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  RobotStatePacket snap{};
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    snap         = latest_state_;
    state_fresh_ = false;
  }

  // Always write to state interfaces so ros2_control sees valid (possibly
  // zero/stale) data even before the hardware connects.  Skipping this when
  // no fresh packet arrived caused joint_state_broadcaster to time out on
  // activation in Jazzy because it waited for at least one read cycle.
  const uint32_t n = std::min(snap.num_joints, static_cast<uint32_t>(n_joints_));
  for (uint32_t i = 0; i < n; ++i) {
    pos_state_[i] = static_cast<double>(snap.joint_pos[i]);
    vel_state_[i] = static_cast<double>(snap.joint_vel[i]);
    eff_state_[i] = static_cast<double>(snap.joint_torque[i]);
  }

  imu_orientation_[0] = static_cast<double>(snap.imu_quat[0]);  // w
  imu_orientation_[1] = static_cast<double>(snap.imu_quat[1]);  // x
  imu_orientation_[2] = static_cast<double>(snap.imu_quat[2]);  // y
  imu_orientation_[3] = static_cast<double>(snap.imu_quat[3]);  // z

  imu_angular_vel_[0] = static_cast<double>(snap.imu_gyro[0]);
  imu_angular_vel_[1] = static_cast<double>(snap.imu_gyro[1]);
  imu_angular_vel_[2] = static_cast<double>(snap.imu_gyro[2]);

  imu_linear_acc_[0] = static_cast<double>(snap.imu_accel[0]);
  imu_linear_acc_[1] = static_cast<double>(snap.imu_accel[1]);
  imu_linear_acc_[2] = static_cast<double>(snap.imu_accel[2]);

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write — package command interfaces into JointCommandPacket and send via UDP
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::return_type
LeggedHW::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // Only send commands when at least one joint has non-zero gains.
  // When running in observation-only mode (just broadcasters, no motion
  // controller), all kp/kd stay at 0 and we leave MuJoCo's internal PD
  // hold active instead of clobbering it with zero-torque packets.
  bool has_active_cmd = false;
  for (std::size_t i = 0; i < n_joints_; ++i) {
    if (kp_cmd_[i] != 0.0 || kd_cmd_[i] != 0.0) {
      has_active_cmd = true;
      break;
    }
  }
  if (!has_active_cmd) return hardware_interface::return_type::OK;

  JointCommandPacket pkt{};
  pkt.timestamp_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  pkt.num_joints = static_cast<uint32_t>(n_joints_);

  for (std::size_t i = 0; i < n_joints_; ++i) {
    pkt.pos_des  [i] = static_cast<float>(pos_cmd_[i]);
    pkt.vel_des  [i] = static_cast<float>(vel_cmd_[i]);
    pkt.torque_ff[i] = static_cast<float>(eff_cmd_[i]);
    pkt.kp       [i] = static_cast<float>(kp_cmd_[i]);
    pkt.kd       [i] = static_cast<float>(kd_cmd_[i]);
  }

  send_sock_.send(&pkt, sizeof(pkt));
  return hardware_interface::return_type::OK;
}

}  // namespace legged_hw

PLUGINLIB_EXPORT_CLASS(legged_hw::LeggedHW, hardware_interface::SystemInterface)
