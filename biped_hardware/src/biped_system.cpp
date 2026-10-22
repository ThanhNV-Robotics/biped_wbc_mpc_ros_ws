#include "biped_hardware/biped_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <string>

#include "biped_hardware/ankle_kinematics.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace biped_hardware
{

namespace
{

// MD80 quickStatus bits (CANdle-SDK MDStatus.hpp); bits 0..6 flag an error in that group
constexpr std::array<const char *, 7> kQuickStatusNames = {
  "main encoder", "output encoder", "calibration encoder", "mosfet bridge",
  "hardware", "communication", "motion"};

constexpr int kMotorCanId[WBC_NUM_MOTORS] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120};

std::string param_or(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & def)
{
  const auto it = params.find(key);
  return it != params.end() ? it->second : def;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

BipedSystem::CallbackReturn BipedSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    board_ip_ = param_or(info_.hardware_parameters, "board_ip", board_ip_);
    board_port_ = static_cast<uint16_t>(
      std::stoi(param_or(info_.hardware_parameters, "board_port", std::to_string(board_port_))));
    timeout_cycles_ = std::stoi(
      param_or(info_.hardware_parameters, "timeout_cycles", std::to_string(timeout_cycles_)));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Invalid hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != WBC_NUM_MOTORS) {
    RCLCPP_FATAL(
      logger_, "Expected %u joints, URDF has %zu", WBC_NUM_MOTORS, info_.joints.size());
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size());
  std::set<int> used_motors;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & ji = info_.joints[i];
    auto & jc = joints_[i];
    jc.name = ji.name;
    try {
      jc.motor_index = std::stoi(ji.parameters.at("motor_index"));
      jc.direction = std::stod(param_or(ji.parameters, "direction", "1"));
    } catch (const std::exception &) {
      RCLCPP_FATAL(
        logger_, "Joint '%s': missing or invalid 'motor_index' / 'direction'", jc.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (jc.motor_index < 0 || jc.motor_index >= static_cast<int>(WBC_NUM_MOTORS) ||
      !used_motors.insert(jc.motor_index).second)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s': motor_index %d out of range or used twice",
        jc.name.c_str(), jc.motor_index);
      return CallbackReturn::ERROR;
    }
    if (std::fabs(std::fabs(jc.direction) - 1.0) > 1e-9) {
      RCLCPP_FATAL(logger_, "Joint '%s': direction must be 1 or -1", jc.name.c_str());
      return CallbackReturn::ERROR;
    }

    if (jc.name == "left_ankle_roll_joint") {ankle_left_.roll = static_cast<int>(i);}
    if (jc.name == "left_ankle_pitch_joint") {ankle_left_.pitch = static_cast<int>(i);}
    if (jc.name == "right_ankle_roll_joint") {ankle_right_.roll = static_cast<int>(i);}
    if (jc.name == "right_ankle_pitch_joint") {ankle_right_.pitch = static_cast<int>(i);}
  }

  if (ankle_left_.roll < 0 || ankle_left_.pitch < 0 ||
    ankle_right_.roll < 0 || ankle_right_.pitch < 0)
  {
    RCLCPP_FATAL(
      logger_, "URDF must contain {left,right}_ankle_{roll,pitch}_joint for the ankle mapping");
    return CallbackReturn::ERROR;
  }

  const std::size_t n = joints_.size();
  position_.assign(n, 0.0);
  velocity_.assign(n, 0.0);
  effort_.assign(n, 0.0);
  position_command_.assign(n, 0.0);
  velocity_command_.assign(n, 0.0);
  effort_command_.assign(n, 0.0);
  kp_command_.assign(n, 0.0);
  kd_command_.assign(n, 0.0);

  RCLCPP_INFO(
    logger_, "Initialized %zu joints, board %s:%u, timeout %d cycles",
    n, board_ip_.c_str(), board_port_, timeout_cycles_);
  return CallbackReturn::SUCCESS;
}

BipedSystem::CallbackReturn BipedSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!udp_.open(board_ip_, board_port_)) {
    RCLCPP_ERROR(logger_, "UDP open failed: %s", udp_.error().c_str());
    return CallbackReturn::ERROR;
  }

  // Handshake: disabled packets every 2 ms until the board answers (max 200 ms)
  bool got_reply = false;
  for (int attempt = 0; attempt < 100 && !got_reply; ++attempt) {
    send_disabled(1);
    if (udp_.wait_readable(2)) {
      got_reply = udp_.recv_latest(state_);
    }
  }
  if (!got_reply) {
    RCLCPP_ERROR(
      logger_, "No reply from %s:%u within 200 ms (%s). Check the cable, the PC address "
      "192.168.10.1/24 and 'ip route get %s'.",
      board_ip_.c_str(), board_port_,
      udp_.error().empty() ? "no data" : udp_.error().c_str(), board_ip_.c_str());
    udp_.close();
    return CallbackReturn::ERROR;
  }

  motor_state_to_joints(state_);
  report_status(state_);
  missed_cycles_ = 0;
  RCLCPP_INFO(logger_, "Board %s:%u answers", board_ip_.c_str(), board_port_);
  return CallbackReturn::SUCCESS;
}

BipedSystem::CallbackReturn BipedSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  udp_.close();
  return CallbackReturn::SUCCESS;
}

BipedSystem::CallbackReturn BipedSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Drop replies that piled up since configure, so the first read() is fresh
  WbcStatePacket stale;
  udp_.recv_latest(stale);

  // Hold the current pose with zero gains until a controller writes something
  position_command_ = position_;
  std::fill(velocity_command_.begin(), velocity_command_.end(), 0.0);
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);
  std::fill(kp_command_.begin(), kp_command_.end(), 0.0);
  std::fill(kd_command_.begin(), kd_command_.end(), 0.0);

  missed_cycles_ = 0;
  RCLCPP_WARN(logger_, "Activated in READ-ONLY mode: motors are never enabled (step 2)");
  return CallbackReturn::SUCCESS;
}

BipedSystem::CallbackReturn BipedSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // write() is not called any more, so disable explicitly instead of waiting for the board timeout
  send_disabled(3);
  return CallbackReturn::SUCCESS;
}

BipedSystem::CallbackReturn BipedSystem::on_error(const rclcpp_lifecycle::State &)
{
  send_disabled(3);
  udp_.close();
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// Interfaces
// ─────────────────────────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface> BipedSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & name = joints_[i].name;
    ifaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &position_[i]);
    ifaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
    ifaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &effort_[i]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface> BipedSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & name = joints_[i].name;
    ifaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &position_command_[i]);
    ifaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &velocity_command_[i]);
    ifaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &effort_command_[i]);
    ifaces.emplace_back(name, "kp", &kp_command_[i]);
    ifaces.emplace_back(name, "kd", &kd_command_[i]);
  }
  return ifaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::return_type BipedSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!udp_.recv_latest(state_)) {
    ++missed_cycles_;
    if (missed_cycles_ >= timeout_cycles_) {
      RCLCPP_ERROR(
        logger_, "No reply from the board for %d cycles%s%s", missed_cycles_,
        udp_.error().empty() ? "" : ": ", udp_.error().c_str());
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;  // keep the last state for a few cycles
  }

  missed_cycles_ = 0;
  motor_state_to_joints(state_);
  report_status(state_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BipedSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Step 2: never enable. The packet is a keep-alive so the board answers every cycle.
  send_disabled(1);
  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

void BipedSystem::send_disabled(int count)
{
  cmd_ = WbcCommandPacket{};
  cmd_.magic = WBC_CMD_MAGIC;
  cmd_.version = WBC_VERSION;
  cmd_.flags = 0;  // ENABLE cleared: the board disables all motors
  for (int i = 0; i < count; ++i) {
    cmd_.seq = ++seq_;
    cmd_.pcTimeUs = static_cast<uint32_t>(steady_clock_.now().nanoseconds() / 1000);
    if (!udp_.send(cmd_)) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 1000, "UDP send failed: %s", udp_.error().c_str());
    }
  }
}

void BipedSystem::motor_state_to_joints(const WbcStatePacket & st)
{
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & m = st.motors[joints_[i].motor_index];
    position_[i] = joints_[i].direction * m.pos;
    velocity_[i] = joints_[i].direction * m.vel;
    effort_[i] = joints_[i].direction * m.tau;
  }

  // Parallel ankles: the two motor values (already sign-corrected above) -> roll / pitch.
  // Velocity and torque use the motor Jacobian at the new roll / pitch.
  const auto map_ankle = [this](const AnklePair & a, bool left) {
      const double t5 = position_[a.roll], t6 = position_[a.pitch];
      const double t5d = velocity_[a.roll], t6d = velocity_[a.pitch];
      const double t5e = effort_[a.roll], t6e = effort_[a.pitch];

      const auto rp = left ? ankle_kinematics::leftJointAngles(t5, t6) :
        ankle_kinematics::rightJointAngles(t5, t6);
      const auto rpd = ankle_kinematics::footVelFromMotor(rp[0], rp[1], t5d, t6d, left);
      const auto rpt = ankle_kinematics::ankleTorqueFromMotor(rp[0], rp[1], t5e, t6e, left);

      position_[a.roll] = rp[0];
      position_[a.pitch] = rp[1];
      velocity_[a.roll] = rpd[0];
      velocity_[a.pitch] = rpd[1];
      effort_[a.roll] = rpt[0];
      effort_[a.pitch] = rpt[1];
    };
  map_ankle(ankle_left_, true);
  map_ankle(ankle_right_, false);
}

void BipedSystem::report_status(const WbcStatePacket & st)
{
  // CM4 needs a few ms after the first command to enter CONTROL and ping the motors, so the
  // first replies are usually placeholders: log every change instead of only the first one
  const bool cm4_data = !(st.status & WBC_ST_NO_CM4_DATA);
  if (cm4_data != cm4_data_) {
    if (cm4_data) {
      RCLCPP_INFO(logger_, "CM4 motor states arriving (board in CONTROL mode)");
    } else {
      RCLCPP_WARN(
        logger_, "Board reports NO_CM4_DATA: states are placeholders (CM4 control loop not running)");
    }
    cm4_data_ = cm4_data;
  }
  if (st.status & WBC_ST_CMD_TIMEOUT) {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 1000, "Board reports CMD_TIMEOUT");
  }
  if (st.status & WBC_ST_NO_CM4_DATA) {
    return;  // per-motor fields are not meaningful without CM4
  }

  // One throttled message for all motors (a throttle per call site would hide all but one)
  std::string report;
  for (std::size_t k = 0; k < WBC_NUM_MOTORS; ++k) {
    const auto & m = st.motors[k];
    std::string problems;
    if (!(m.flags & WBC_MOT_ONLINE)) {problems += " offline";}
    if (m.flags & WBC_MOT_MISSED) {problems += " missed-reply";}
    for (std::size_t b = 0; b < kQuickStatusNames.size(); ++b) {
      if (m.quickStatus & (1U << b)) {
        problems += std::string(" ") + kQuickStatusNames[b] + "-error";
      }
    }
    if (!problems.empty()) {
      report += "\n  MD " + std::to_string(kMotorCanId[k]) + ":" + problems +
        " (" + std::to_string(m.temperature) + " degC)";
    }
  }
  if (!report.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 1000, "Motor problems:%s", report.c_str());
  }
}

}  // namespace biped_hardware

PLUGINLIB_EXPORT_CLASS(biped_hardware::BipedSystem, hardware_interface::SystemInterface)
