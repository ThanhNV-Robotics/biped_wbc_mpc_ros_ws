#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco_ros2_control
{

/// Emulates the real biped's motor drives and board (biped_hardware/BipedSystem + NUCLEO firmware)
/// in MuJoCo. Used by MujocoSystemInterface when the hardware parameter `pvt_mode` is true.
///
/// Each joint takes PVT commands (position, velocity, feed-forward torque). The drive applies
///   tau = kp * (p_cmd - q) + kd * (v_cmd - dq) + tau_ff
/// with fixed per-joint gains (joint params kp / kd, from biped_hardware/config/joint_gains.yaml),
/// evaluated in the physics thread before every mj_step: the real motors run their PD loop on the
/// drive, much faster than the 500 Hz command rate. Joint space only: the MJCF ankle is a serial
/// roll / pitch chain (the parallel ankle linkage is not simulated).
///
/// States, as the board reports them:
///   DISABLED  zero torque (motors off): at start and after disable()
///   ENABLED   PVT law with the gains
///   DAMPING   command older than command_timeout, or a non-finite command: kp = 0,
///             kd = damping_kd, no torque for damping_duration, then DISABLED (as the firmware)
///
/// Torque mode (set_torque_mode): ENABLED applies tau = tau_ff only (kp = kd = 0), as the numerical
/// TSID simulation (tsid_examples) applies its torques. Set while a controller claims only the
/// effort commands; off again (PVT, holding the current pose) when it releases them.
class BipedPvtDrive
{
public:
  enum class State
  {
    DISABLED,
    ENABLED,
    DAMPING
  };

  struct Params
  {
    double command_timeout{ 0.02 };  // s of sim time, 0 = no timeout (firmware: 20 ms)
    double damping_kd{ 1.0 };        // Nm*s/rad (firmware: kd 1.0)
    double damping_duration{ 1.0 };  // s (firmware: 1 s)
  };

  struct JointConfig
  {
    std::string name;
    double kp{ 0.0 };
    double kd{ 0.0 };
    int qpos_adr{ -1 };
    int qvel_adr{ -1 };
    int actuator_id{ -1 };
    bool limited{ false };
    double lower{ 0.0 };  // rad, MJCF joint range (same as the URDF limits)
    double upper{ 0.0 };
  };

  /// Joints in ros2_control order. Returns an empty string, or why the configuration is invalid.
  std::string configure(const mjModel* m, std::vector<JointConfig> joints, const Params& params);

  const std::vector<JointConfig>& joints() const
  {
    return joints_;
  }
  State state() const
  {
    return state_.load();
  }

  // ── ros2_control thread ──
  /// Latest PVT commands (joints() order) and the sim time they were sent at
  void set_commands(const std::vector<double>& pos, const std::vector<double>& vel, const std::vector<double>& eff,
                    double sim_time);
  /// Same checks as BipedSystem::on_activate: every joint inside its range. Empty = may enable.
  std::string activation_blocker(const std::vector<double>& q) const;
  /// ENABLED, holding q (targets = q, velocity 0, torque 0)
  void enable(const std::vector<double>& q, double sim_time);
  /// DISABLED at once (zero torque)
  void disable();
  /// true: tau = tau_ff only. false: PVT law, targets reset to q (hold the current pose)
  void set_torque_mode(bool on, const std::vector<double>& q, double sim_time);
  bool torque_mode() const
  {
    return torque_mode_.load();
  }
  /// Why the drive left ENABLED on its own (timeout / non-finite command); cleared by enable()
  std::string fault() const;

  // ── physics thread, before every mj_step ──
  /// Writes d->ctrl of the drive's actuators from the state and d->qpos / d->qvel
  void apply(mjData* d);

private:
  struct Setpoint
  {
    std::vector<double> pos, vel, eff;
    double time{ 0.0 };
    bool finite{ true };
  };

  std::vector<JointConfig> joints_;
  Params params_;

  mutable std::mutex mutex_;  // guards cmd_ and fault_
  Setpoint cmd_;              // written by the ros2_control thread
  std::string fault_;

  Setpoint sp_;               // physics thread copy of cmd_ (preallocated, no allocation in apply)
  std::atomic<State> state_{ State::DISABLED };
  std::atomic<bool> torque_mode_{ false };
  double damping_start_{ 0.0 };
};

}  // namespace mujoco_ros2_control
