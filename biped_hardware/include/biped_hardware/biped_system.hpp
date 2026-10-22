#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
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
/// One WbcCommandPacket per cycle, the reply is picked up by the next read().
/// Joint space (URDF) <-> motor space (wbc_protocol.h motors[] index):
///   - hips / knees: motor = direction * joint (position, velocity, torque)
///   - ankles: the roll joint carries the upper motor (50 / 110), the pitch joint the lower
///     motor (60 / 120); ankle_kinematics.hpp maps position, velocity and torque.
/// Gains: per-joint kp / kd from config/joint_gains.yaml (URDF joint params, via the xacro). Sent
/// with GAINS_VALID only in the enable handshake: the firmware writes them to the motors, reads
/// them back and then enables. While active the gains are fixed: PVT commands only (ankles:
/// motor-space gains of the motor each ankle joint carries).
///
/// Lifecycle (the hardware starts INACTIVE, see config/controllers.yaml):
///   inactive  states are read, motors disabled (read() sends ENABLE = 0 keep-alives)
///   activate  checks (12 motors online, no fault, every joint inside its URDF limits), then
///             ENABLE with targets = current pose, waits for WBC_ST_ENABLED (motors hold the pose)
///   active    write() sends the joint PVT commands with ENABLE every cycle; read()
///             returns ERROR if the board leaves ENABLED (timeout / fault / damping)
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
    int motor_index{-1};    // index into WbcCommandPacket / WbcStatePacket motors[]
    double direction{1.0};  // +1 / -1, applied in motor space
    double lower{0.0};      // URDF <limit>, used as the start range on activation
    double upper{0.0};
    double kp{0.0};         // impedance gains written to the motor at activation (motor space)
    double kd{0.0};
  };

  struct AnklePair
  {
    int roll{-1};   // joint index of *_ankle_roll_joint (upper motor)
    int pitch{-1};  // joint index of *_ankle_pitch_joint (lower motor)
  };

  /// Loop timing of the controller_manager thread, logged every kTimingWindowS seconds.
  /// period = read() to read(); compute = end of read() to start of write() (the controllers,
  /// later the WBC). Fixed-size histogram, no allocation in the loop.
  struct TimingStats
  {
    static constexpr int kBins = 400;          // 0.1 ms bins, last bin = 40 ms and more
    std::array<uint32_t, kBins> period_hist{};
    uint32_t cycles{0};
    double period_sum_ms{0.0}, period_max_ms{0.0};
    double compute_sum_ms{0.0}, compute_max_ms{0.0};
    double read_max_ms{0.0}, write_max_ms{0.0};
    uint32_t over_2_5{0}, over_5{0}, over_10{0}, over_20{0};
    uint32_t no_reply{0}, cmd_timeout{0};
    std::array<uint32_t, WBC_NUM_MOTORS> missed{};   // state replies with MISSED, per motor
  };
  static constexpr double kTimingWindowS = 5.0;

  bool load_urdf_limits();
  void send_packet(uint16_t flags);
  void send_disabled(int count);
  void joints_to_motor_commands(WbcCommandPacket & cmd) const;
  void motor_state_to_joints(const WbcStatePacket & st);
  void report_status(const WbcStatePacket & st);
  bool exchange_until(uint16_t flags, int max_cycles, bool (BipedSystem::*done)() const);
  bool board_enabled() const;
  bool has_motor_data() const;
  std::string activation_blocker() const;
  void log_motor_status() const;
  void log_thread_scheduling();
  void report_timing(std::chrono::steady_clock::time_point now);

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

  // Commands, joint space
  std::vector<double> position_command_;
  std::vector<double> velocity_command_;
  std::vector<double> effort_command_;

  // Link to the board. on_activate / on_deactivate may run in another thread than
  // read() / write(); the loop only try_locks and skips a cycle while a transition runs.
  std::mutex link_mutex_;
  UdpClient udp_;
  WbcCommandPacket cmd_{};
  WbcStatePacket state_{};
  uint32_t seq_{0};
  int missed_cycles_{0};
  bool cm4_data_{true};  // last seen !NO_CM4_DATA; starts true so placeholders are reported
  // A single missed reply is normal (lost CAN arbitration, the command still arrived): only
  // counted in the timing line. Warn when a motor misses this many replies in a row.
  static constexpr uint16_t kMissedWarnInRow = 3;
  std::array<uint16_t, WBC_NUM_MOTORS> consec_missed_{};
  bool active_{false};   // lifecycle ACTIVE: write() sends real commands
  double enabled_state_{0.0};  // state interface "<hardware name>/enabled": 1 while motors enabled
  mutable bool warned_nonfinite_{false};  // set from the const command mapping

  // Loop timing
  TimingStats timing_;
  std::chrono::steady_clock::time_point last_read_start_{};
  std::chrono::steady_clock::time_point read_end_{};
  std::chrono::steady_clock::time_point timing_window_start_{};
  bool timing_started_{false};
  bool sched_logged_{false};
};

}  // namespace biped_hardware
