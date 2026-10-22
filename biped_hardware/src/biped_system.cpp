#include "biped_hardware/biped_system.hpp"

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <thread>

#include "biped_hardware/ankle_kinematics.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

namespace biped_hardware
{

namespace
{

// MD80 quickStatus bits (CANdle-SDK MDStatus.hpp); bits 0..6 flag an error in that group
constexpr std::array<const char *, 7> kQuickStatusNames = {
  "main encoder", "output encoder", "calibration encoder", "mosfet bridge",
  "hardware", "communication", "motion"};
constexpr uint16_t kQuickStatusErrors = 0x007F;

constexpr int kMotorCanId[WBC_NUM_MOTORS] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120};

// Sanity caps for config/joint_gains.yaml (catch typos, not a tuning limit)
constexpr double kMaxKp = 1000.0;
constexpr double kMaxKd = 50.0;

// Activation: ENABLE = 0 phase (also the edge the firmware needs), then wait for ENABLED
constexpr int kDisabledCycles = 150;       // 300 ms at 2 ms (time to leave MAINTENANCE + ping)
constexpr int kEnableWaitCycles = 500;     // 1 s
constexpr auto kHandshakePeriod = std::chrono::milliseconds(2);

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
      jc.kp = std::stod(ji.parameters.at("kp"));
      jc.kd = std::stod(ji.parameters.at("kd"));
    } catch (const std::exception &) {
      RCLCPP_FATAL(
        logger_, "Joint '%s': missing or invalid 'motor_index' / 'direction' / 'kp' / 'kd' "
        "(gains come from config/joint_gains.yaml)", jc.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (!(jc.kp >= 0.0 && jc.kp <= kMaxKp) || !(jc.kd >= 0.0 && jc.kd <= kMaxKd)) {
      RCLCPP_FATAL(
        logger_, "Joint '%s': kp %.3f / kd %.3f outside [0, %.0f] / [0, %.0f]",
        jc.name.c_str(), jc.kp, jc.kd, kMaxKp, kMaxKd);
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

  if (!load_urdf_limits()) {
    return CallbackReturn::ERROR;
  }

  const std::size_t n = joints_.size();
  position_.assign(n, 0.0);
  velocity_.assign(n, 0.0);
  effort_.assign(n, 0.0);
  position_command_.assign(n, 0.0);
  velocity_command_.assign(n, 0.0);
  effort_command_.assign(n, 0.0);

  RCLCPP_INFO(
    logger_, "Initialized %zu joints, board %s:%u, timeout %d cycles",
    n, board_ip_.c_str(), board_port_, timeout_cycles_);
  return CallbackReturn::SUCCESS;
}

bool BipedSystem::load_urdf_limits()
{
  // info_.original_xml is the robot description the hardware was created from
  urdf::Model model;
  if (!model.initString(info_.original_xml)) {
    RCLCPP_FATAL(logger_, "Could not parse the URDF to read the joint limits (start range)");
    return false;
  }
  for (auto & jc : joints_) {
    const auto joint = model.getJoint(jc.name);
    if (!joint || !joint->limits || joint->type != urdf::Joint::REVOLUTE) {
      RCLCPP_FATAL(logger_, "Joint '%s': no revolute joint with <limit> in the URDF", jc.name.c_str());
      return false;
    }
    jc.lower = joint->limits->lower;
    jc.upper = joint->limits->upper;
  }
  return true;
}

BipedSystem::CallbackReturn BipedSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(link_mutex_);
  if (!udp_.open(board_ip_, board_port_)) {
    RCLCPP_ERROR(logger_, "UDP open failed: %s", udp_.error().c_str());
    return CallbackReturn::ERROR;
  }

  // 1. Route: the board is on a direct cable. If the wired port is down, Linux sends the packets
  //    through the default route (e.g. Wi-Fi) to some other machine.
  const std::string local = udp_.local_address();
  const auto subnet = [](const std::string & ip) {return ip.substr(0, ip.rfind('.'));};
  if (local.empty() || subnet(local) != subnet(board_ip_)) {
    RCLCPP_ERROR(
      logger_, "Network: %s would be reached from %s, not from the wired port (%s.x). Is the "
      "Ethernet cable plugged in and the PC port up with 192.168.10.1/24? Check: ip -br addr, "
      "ip route get %s", board_ip_.c_str(), local.empty() ? "?" : local.c_str(),
      subnet(board_ip_).c_str(), board_ip_.c_str());
    udp_.close();
    return CallbackReturn::ERROR;
  }

  // 2. Link: 50 disabled packets, one reply each, round-trip time per packet
  constexpr int kPings = 50;
  int replies = 0;
  double rtt_sum_ms = 0.0, rtt_max_ms = 0.0;
  for (int k = 0; k < kPings; ++k) {
    const auto t0 = std::chrono::steady_clock::now();
    send_disabled(1);
    if (udp_.wait_readable(5) && udp_.recv_latest(state_)) {
      const double rtt_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      ++replies;
      rtt_sum_ms += rtt_ms;
      rtt_max_ms = std::max(rtt_max_ms, rtt_ms);
    }
    std::this_thread::sleep_for(kHandshakePeriod);
  }
  if (replies == 0) {
    RCLCPP_ERROR(
      logger_, "Network: no reply from %s:%u (%s). Is the board powered and flashed? Its serial "
      "log should show '[UDP] listening on 192.168.10.10:9999'.",
      board_ip_.c_str(), board_port_, udp_.error().empty() ? "no data" : udp_.error().c_str());
    udp_.close();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger_, "Network OK: board %s:%u via %s, %d/%d replies, RTT avg %.2f max %.2f ms",
    board_ip_.c_str(), board_port_, local.c_str(), replies, kPings, rtt_sum_ms / replies, rtt_max_ms);
  if (replies < kPings * 9 / 10) {
    RCLCPP_WARN(logger_, "Network: %d of %d packets lost", kPings - replies, kPings);
  }

  // 3. Motor data: after the first commands the board switches to CONTROL and pings the motors
  //    (up to ~60 ms with motors missing). Then a few more cycles so the values are fresh.
  if (!exchange_until(0, 500, &BipedSystem::has_motor_data)) {
    RCLCPP_ERROR(
      logger_, "Board answers but sends no motor data within 1 s (NO_CM4_DATA): CM4 is not in "
      "CONTROL mode. Is nucleotool still using the board? See the board's serial log.");
    udp_.close();
    return CallbackReturn::ERROR;
  }
  exchange_until(0, 25, nullptr);

  // 4. Motor status: logged here; activation refuses the same problems later
  report_status(state_);
  log_motor_status();
  missed_cycles_ = 0;
  active_ = false;
  return CallbackReturn::SUCCESS;
}

void BipedSystem::log_motor_status() const
{
  std::string table;
  int online = 0;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    const auto & m = state_.motors[j.motor_index];
    const bool is_online = (m.flags & WBC_MOT_ONLINE) != 0;
    const bool inside = position_[i] >= j.lower && position_[i] <= j.upper;
    online += is_online ? 1 : 0;
    std::string faults;
    for (std::size_t b = 0; b < kQuickStatusNames.size(); ++b) {
      if (m.quickStatus & (1U << b)) {faults += std::string(" ") + kQuickStatusNames[b];}
    }
    char line[200];
    std::snprintf(
      line, sizeof(line),
      "\n  %-24s MD %3d  %-7s %+7.3f rad [%+.3f, %+.3f] %-7s %3u degC  kp %6.1f kd %5.2f%s%s",
      j.name.c_str(), kMotorCanId[j.motor_index], is_online ? "online" : "OFFLINE",
      position_[i], j.lower, j.upper, inside ? "ok" : "OUTSIDE", m.temperature, j.kp, j.kd,
      faults.empty() ? "" : "  faults:", faults.c_str());
    table += line;
  }
  RCLCPP_INFO(logger_, "Motor status (%d/%zu online):%s", online, joints_.size(), table.c_str());

  std::string zero_kp;
  for (const auto & j : joints_) {
    if (j.kp == 0.0) {zero_kp += " " + j.name;}
  }
  if (!zero_kp.empty()) {
    RCLCPP_WARN(
      logger_, "kp = 0 in config/joint_gains.yaml for:%s. These joints will be limp when enabled.",
      zero_kp.c_str());
  }

  const std::string why = activation_blocker();
  if (why.empty()) {
    RCLCPP_INFO(logger_, "Configured: all motors online, all joints inside their limits. Ready to enable.");
  } else {
    RCLCPP_WARN(logger_, "Configured, but activation will be refused until fixed:%s", why.c_str());
  }
}

bool BipedSystem::has_motor_data() const
{
  return (state_.status & WBC_ST_NO_CM4_DATA) == 0;
}

BipedSystem::CallbackReturn BipedSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(link_mutex_);
  udp_.close();
  return CallbackReturn::SUCCESS;
}

BipedSystem::CallbackReturn BipedSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(link_mutex_);

  // 1. ENABLE = 0 for 300 ms: fresh states, and the edge the firmware needs before ENABLE
  if (!exchange_until(0, kDisabledCycles, nullptr)) {
    RCLCPP_ERROR(logger_, "Activation refused: no reply from the board");
    return CallbackReturn::ERROR;
  }
  const std::string why = activation_blocker();
  if (!why.empty()) {
    RCLCPP_ERROR(logger_, "Activation refused, motors stay disabled: %s", why.c_str());
    send_disabled(3);
    return CallbackReturn::ERROR;
  }

  // 2. Targets = current pose, no velocity / torque: with the configured gains the motors hold it
  position_command_ = position_;
  std::fill(velocity_command_.begin(), velocity_command_.end(), 0.0);
  std::fill(effort_command_.begin(), effort_command_.end(), 0.0);

  // 3. ENABLE | GAINS_VALID until the board reports ENABLED. The firmware writes the gains from
  //    config/joint_gains.yaml, reads them back, then enables the 12 motors (~40 ms).
  if (!exchange_until(
      WBC_CMD_ENABLE | WBC_CMD_GAINS_VALID, kEnableWaitCycles, &BipedSystem::board_enabled))
  {
    send_disabled(3);
    RCLCPP_ERROR(
      logger_, "Activation failed: the board did not enable within 1 s (status 0x%04x). "
      "The board's serial log says why ('[WBC] ENABLE refused: ...').", state_.status);
    return CallbackReturn::ERROR;
  }

  missed_cycles_ = 0;
  warned_nonfinite_ = false;
  timing_ = TimingStats{};   // don't count the activation in the loop timing
  timing_started_ = false;
  active_ = true;
  enabled_state_ = 1.0;
  RCLCPP_WARN(
    logger_, "ACTIVE: 12 motors enabled with the gains from config/joint_gains.yaml (written and "
    "verified by the board), holding the current pose. Gains fixed, PVT commands only.");
  return CallbackReturn::SUCCESS;
}

BipedSystem::CallbackReturn BipedSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(link_mutex_);
  active_ = false;
  enabled_state_ = 0.0;
  // ENABLE = 0 at once; read() keeps sending ENABLE = 0 keep-alives while inactive
  send_disabled(3);
  RCLCPP_INFO(logger_, "Deactivated: motors disabled");
  return CallbackReturn::SUCCESS;
}

BipedSystem::CallbackReturn BipedSystem::on_error(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(link_mutex_);
  active_ = false;
  enabled_state_ = 0.0;
  send_disabled(3);
  udp_.close();
  RCLCPP_ERROR(logger_, "Error state: motors disabled, link closed");
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
  // 1 while the motors are enabled. Controllers use it to keep their targets at the measured pose
  // while the motors are off, so nothing jumps when they are enabled.
  ifaces.emplace_back(info_.name, "enabled", &enabled_state_);
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
  }
  return ifaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────────────────────

hardware_interface::return_type BipedSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::unique_lock<std::mutex> lock(link_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return hardware_interface::return_type::OK;   // a lifecycle transition is using the link
  }

  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  if (!sched_logged_) {
    log_thread_scheduling();
    sched_logged_ = true;
  }
  if (timing_started_) {
    const double period_ms = std::chrono::duration<double, std::milli>(start - last_read_start_).count();
    auto & t = timing_;
    ++t.cycles;
    t.period_sum_ms += period_ms;
    t.period_max_ms = std::max(t.period_max_ms, period_ms);
    ++t.period_hist[std::min(static_cast<int>(period_ms * 10.0), TimingStats::kBins - 1)];
    if (period_ms > 2.5) {++t.over_2_5;}
    if (period_ms > 5.0) {++t.over_5;}
    if (period_ms > 10.0) {++t.over_10;}
    if (period_ms > 20.0) {++t.over_20;}
  } else {
    timing_started_ = true;
    timing_window_start_ = start;
  }
  last_read_start_ = start;

  const bool got = udp_.recv_latest(state_);
  if (!got) {
    ++timing_.no_reply;
    ++missed_cycles_;
    if (missed_cycles_ >= timeout_cycles_) {
      RCLCPP_ERROR(
        logger_, "No reply from the board for %d cycles%s%s", missed_cycles_,
        udp_.error().empty() ? "" : ": ", udp_.error().c_str());
      return hardware_interface::return_type::ERROR;
    }
    // keep the last state for a few cycles
  } else {
    missed_cycles_ = 0;
    if (state_.status & WBC_ST_CMD_TIMEOUT) {++timing_.cmd_timeout;}
    motor_state_to_joints(state_);
    report_status(state_);
    // Active, but the board no longer follows us (timeout, fault, damping): stop
    if (active_ && !(state_.status & WBC_ST_ENABLED)) {
      enabled_state_ = 0.0;
      RCLCPP_ERROR(
        logger_, "Board left ENABLED (status 0x%04x%s%s): motors are damping / disabled",
        state_.status, (state_.status & WBC_ST_CMD_TIMEOUT) ? ", CMD_TIMEOUT" : "",
        (state_.status & WBC_ST_MOTOR_FAULT) ? ", MOTOR_FAULT" : "");
      return hardware_interface::return_type::ERROR;
    }
  }

  // Inactive: write() is not called, so keep the board answering from here
  if (!active_) {
    send_disabled(1);
  }

  read_end_ = Clock::now();
  timing_.read_max_ms = std::max(
    timing_.read_max_ms, std::chrono::duration<double, std::milli>(read_end_ - start).count());
  report_timing(read_end_);   // here, not in write(): write() does not run while inactive
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BipedSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::unique_lock<std::mutex> lock(link_mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !active_) {
    return hardware_interface::return_type::OK;
  }

  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  if (timing_started_) {
    const double compute_ms = std::chrono::duration<double, std::milli>(start - read_end_).count();
    timing_.compute_sum_ms += compute_ms;
    timing_.compute_max_ms = std::max(timing_.compute_max_ms, compute_ms);
  }

  // PVT only: the motors keep their own kp / kd (the firmware also ignores gains, WBC_WRITE_GAINS 0)
  send_packet(WBC_CMD_ENABLE);

  const auto end = Clock::now();
  timing_.write_max_ms = std::max(
    timing_.write_max_ms, std::chrono::duration<double, std::milli>(end - start).count());
  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

void BipedSystem::send_packet(uint16_t flags)
{
  cmd_ = WbcCommandPacket{};
  cmd_.magic = WBC_CMD_MAGIC;
  cmd_.version = WBC_VERSION;
  cmd_.flags = flags;
  if (flags & WBC_CMD_ENABLE) {
    joints_to_motor_commands(cmd_);
  }
  cmd_.seq = ++seq_;
  cmd_.pcTimeUs = static_cast<uint32_t>(steady_clock_.now().nanoseconds() / 1000);
  if (!udp_.send(cmd_)) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 1000, "UDP send failed: %s", udp_.error().c_str());
  }
}

void BipedSystem::send_disabled(int count)
{
  // ENABLE cleared, all targets and gains 0: the board disables all motors
  for (int i = 0; i < count; ++i) {
    send_packet(0);
  }
}

void BipedSystem::joints_to_motor_commands(WbcCommandPacket & cmd) const
{
  bool finite = true;
  const auto put = [&](int motor, double pos, double vel, double tau, double kp, double kd) {
      auto & m = cmd.motors[motor];
      m.pos = static_cast<float>(pos);
      m.vel = static_cast<float>(vel);
      m.tau = static_cast<float>(tau);
      m.kp = static_cast<float>(kp);
      m.kd = static_cast<float>(kd);
      finite = finite && std::isfinite(m.pos) && std::isfinite(m.vel) && std::isfinite(m.tau) &&
        std::isfinite(m.kp) && std::isfinite(m.kd);
    };

  // Hips and knees: motor = direction * joint; gains are not signed
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const int ii = static_cast<int>(i);
    if (ii == ankle_left_.roll || ii == ankle_left_.pitch ||
      ii == ankle_right_.roll || ii == ankle_right_.pitch)
    {
      continue;
    }
    const auto & j = joints_[i];
    put(
      j.motor_index, j.direction * position_command_[i], j.direction * velocity_command_[i],
      j.direction * effort_command_[i], j.kp, j.kd);
  }

  // Ankles: roll / pitch -> the two motors (motor order: roll joint's motor first). Position and
  // velocity at the commanded pose, torque with the Jacobian at the measured pose. The commanded
  // pose is first clamped to the mechanical range (roll ±15, pitch ±30 deg) and to the part of it
  // the linkage can reach (roll reduced at large pitch), so the inverse map is always defined.
  const auto map_ankle = [&](const AnklePair & a, bool left) {
      const auto rp = left ?
        ankle_kinematics::leftClampToWorkspace(position_command_[a.roll], position_command_[a.pitch]) :
        ankle_kinematics::rightClampToWorkspace(position_command_[a.roll], position_command_[a.pitch]);
      const double r = rp[0];
      const double p = rp[1];
      const auto th = left ? ankle_kinematics::leftMotorAngles(r, p) :
        ankle_kinematics::rightMotorAngles(r, p);
      const auto thd = ankle_kinematics::motorVelFromFoot(
        r, p, velocity_command_[a.roll], velocity_command_[a.pitch], left);
      const auto tau = ankle_kinematics::motorTorqueFromAnkle(
        position_[a.roll], position_[a.pitch], effort_command_[a.roll], effort_command_[a.pitch], left);

      const auto & jr = joints_[a.roll];
      const auto & jp = joints_[a.pitch];
      // Configured gains of the motor each ankle joint carries (motor space)
      put(jr.motor_index, jr.direction * th[0], jr.direction * thd[0], jr.direction * tau[0],
        jr.kp, jr.kd);
      put(jp.motor_index, jp.direction * th[1], jp.direction * thd[1], jp.direction * tau[1],
        jp.kp, jp.kd);
    };
  map_ankle(ankle_left_, true);
  map_ankle(ankle_right_, false);

  if (!finite && !warned_nonfinite_) {
    // The firmware rejects it (damping, then disable); read() then reports the board left ENABLED
    RCLCPP_ERROR(logger_, "Non-finite joint command (NaN / inf): the board will damp and disable");
    warned_nonfinite_ = true;
  }
}

bool BipedSystem::exchange_until(
  uint16_t flags, int max_cycles, bool (BipedSystem::*done)() const)
{
  // Blocking handshake for lifecycle transitions: one packet every 2 ms, newest reply kept
  bool any_reply = false;
  auto next = std::chrono::steady_clock::now();
  for (int k = 0; k < max_cycles; ++k) {
    send_packet(flags);
    next += kHandshakePeriod;
    std::this_thread::sleep_until(next);
    if (udp_.recv_latest(state_)) {
      any_reply = true;
      motor_state_to_joints(state_);
      if (done != nullptr && (this->*done)()) {
        return true;
      }
    }
  }
  return done == nullptr && any_reply;
}

bool BipedSystem::board_enabled() const
{
  return (state_.status & WBC_ST_ENABLED) != 0;
}

std::string BipedSystem::activation_blocker() const
{
  if (state_.status & WBC_ST_NO_CM4_DATA) {
    return "the board has no motor data (CM4 not in CONTROL mode)";
  }
  std::string why;
  for (std::size_t k = 0; k < WBC_NUM_MOTORS; ++k) {
    const auto & m = state_.motors[k];
    if (!(m.flags & WBC_MOT_ONLINE)) {
      why += " MD " + std::to_string(kMotorCanId[k]) + " offline;";
    } else if (m.quickStatus & kQuickStatusErrors) {
      why += " MD " + std::to_string(kMotorCanId[k]) + " quickStatus error;";
    }
  }
  // Ankles: the kinematics clamps roll / pitch to ±0.349 rad, the URDF allows more (pitch ±0.524).
  // Beyond the clamp the joint value is wrong and the motor targets built from it would differ
  // from where the motors are: with the motors' own kp they would jump there on enable. So the
  // targets must reproduce the measured motor positions.
  const auto check_ankle = [&](const AnklePair & a, bool left, const char * side) {
      const auto th = left ?
        ankle_kinematics::leftMotorAngles(position_[a.roll], position_[a.pitch]) :
        ankle_kinematics::rightMotorAngles(position_[a.roll], position_[a.pitch]);
      const auto & jr = joints_[a.roll];
      const auto & jp = joints_[a.pitch];
      const double err = std::max(
        std::fabs(jr.direction * th[0] - state_.motors[jr.motor_index].pos),
        std::fabs(jp.direction * th[1] - state_.motors[jp.motor_index].pos));
      if (!(err < 0.01)) {   // also catches NaN
        char buf[200];
        std::snprintf(
          buf, sizeof(buf), " %s ankle outside the kinematics range (roll %.3f, pitch %.3f rad, "
          "motor targets off by %.3f rad);", side, position_[a.roll], position_[a.pitch], err);
        why += buf;
      }
    };
  check_ankle(ankle_left_, true, "left");
  check_ankle(ankle_right_, false, "right");

  // Single-turn encoders: a joint outside its URDF range is probably read wrong
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    if (position_[i] < j.lower || position_[i] > j.upper) {
      char buf[160];
      std::snprintf(
        buf, sizeof(buf), " %s at %.3f rad, outside [%.3f, %.3f];",
        j.name.c_str(), position_[i], j.lower, j.upper);
      why += buf;
    }
  }
  return why;
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
    // MISSED: counted, warned only after kMissedWarnInRow in a row (once per streak)
    if ((m.flags & WBC_MOT_ONLINE) && (m.flags & WBC_MOT_MISSED)) {
      ++timing_.missed[k];
      if (++consec_missed_[k] == kMissedWarnInRow) {
        problems += " missed " + std::to_string(kMissedWarnInRow) + " replies in a row";
      }
    } else {
      consec_missed_[k] = 0;
    }
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

void BipedSystem::log_thread_scheduling()
{
  // read() runs in the controller_manager loop thread, so this is that thread
  sched_param param{};
  int policy = 0;
  pthread_getschedparam(pthread_self(), &policy, &param);
  const char * name = policy == SCHED_FIFO ? "SCHED_FIFO" :
    policy == SCHED_RR ? "SCHED_RR" : "SCHED_OTHER (not real-time)";
  RCLCPP_INFO(
    logger_, "Control loop thread: %s, priority %d, running on CPU %d", name,
    param.sched_priority, sched_getcpu());
}

void BipedSystem::report_timing(std::chrono::steady_clock::time_point now)
{
  const double window_s = std::chrono::duration<double>(now - timing_window_start_).count();
  if (window_s < kTimingWindowS || timing_.cycles == 0) {
    return;
  }
  auto & t = timing_;

  // p99 of the period from the 0.1 ms histogram
  const uint32_t target = static_cast<uint32_t>(0.99 * t.cycles);
  uint32_t acc = 0;
  int bin = 0;
  for (; bin < TimingStats::kBins - 1; ++bin) {
    acc += t.period_hist[bin];
    if (acc > target) {break;}
  }
  const double p99_ms = (bin + 1) * 0.1;

  std::string missed;
  for (std::size_t k = 0; k < WBC_NUM_MOTORS; ++k) {
    if (t.missed[k] > 0) {
      missed += " " + std::to_string(kMotorCanId[k]) + ":" + std::to_string(t.missed[k]);
    }
  }

  RCLCPP_INFO(
    logger_,
    "Loop %.0f Hz | period avg %.3f p99 %.1f max %.2f ms, >2.5ms %u >5ms %u >10ms %u >20ms %u | "
    "compute avg %.3f max %.3f ms | read max %.3f write max %.3f ms | no reply %u, CMD_TIMEOUT %u%s%s",
    t.cycles / window_s, t.period_sum_ms / t.cycles, p99_ms, t.period_max_ms,
    t.over_2_5, t.over_5, t.over_10, t.over_20,
    t.compute_sum_ms / t.cycles, t.compute_max_ms, t.read_max_ms, t.write_max_ms,
    t.no_reply, t.cmd_timeout, missed.empty() ? "" : " | missed replies MD", missed.c_str());

  timing_ = TimingStats{};
  timing_window_start_ = now;
}

}  // namespace biped_hardware

PLUGINLIB_EXPORT_CLASS(biped_hardware::BipedSystem, hardware_interface::SystemInterface)
