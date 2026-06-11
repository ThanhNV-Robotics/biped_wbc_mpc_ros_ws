//
// ROS2 port of legged_common/hardware_interface/HybridJointInterface.h
// Original by qiayuan (2021/11/5).
//
// ROS1 used HardwareResourceManager<HybridJointHandle> as the interface registry.
// In ROS2 ros2_control that registry is replaced by:
//   - SystemInterface::export_command_interfaces()  (hardware side)
//   - ControllerInterface::command_interface_configuration()  (controller side)
//
// This file provides HybridJointHandle — a per-joint wrapper around the five
// LoanedCommandInterfaces (position, velocity, kp, kd, feedforward_effort) and
// two LoanedStateInterfaces (position, velocity) that the controller holds.
// The controller creates one handle per joint in on_activate() and calls
// setCommand(posDes, velDes, kp, kd, ff) inside update(), exactly as in ROS1.
//
#pragma once

#include <cassert>
#include <functional>
#include <stdexcept>
#include <string>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace legged {

/**
 * Per-joint handle that groups the five command interfaces and two state
 * interfaces that implement the HybridJoint protocol.
 *
 * Required command interfaces (exported by MujocoSystemInterface):
 *   <joint>/position, <joint>/velocity,
 *   <joint>/kp, <joint>/kd, <joint>/feedforward_effort
 *
 * Required state interfaces:
 *   <joint>/position, <joint>/velocity
 *
 * Lifetime: handles must not outlive the LoanedInterface objects they reference
 * (i.e. they must be destroyed before on_deactivate() releases the loans).
 */
class HybridJointHandle
{
public:
  HybridJointHandle() = delete;

  HybridJointHandle(const std::string& name,
                    hardware_interface::LoanedStateInterface& pos_state,
                    hardware_interface::LoanedStateInterface& vel_state,
                    hardware_interface::LoanedCommandInterface& pos_cmd,
                    hardware_interface::LoanedCommandInterface& vel_cmd,
                    hardware_interface::LoanedCommandInterface& kp_cmd,
                    hardware_interface::LoanedCommandInterface& kd_cmd,
                    hardware_interface::LoanedCommandInterface& ff_cmd)
    : name_(name),
      pos_state_(pos_state), vel_state_(vel_state),
      pos_cmd_(pos_cmd), vel_cmd_(vel_cmd),
      kp_cmd_(kp_cmd), kd_cmd_(kd_cmd), ff_cmd_(ff_cmd)
  {}

  // ── Identity ────────────────────────────────────────────────────────────────
  const std::string& getName() const { return name_; }

  // ── State reads (current joint state, set by the hardware) ──────────────────
  double getPosition() const { return pos_state_.get().get_value(); }
  double getVelocity() const { return vel_state_.get().get_value(); }

  // ── Command reads (last value written by the controller) ────────────────────
  double getPositionDesired() const { return pos_cmd_.get().get_value(); }
  double getVelocityDesired() const { return vel_cmd_.get().get_value(); }
  double getKp()              const { return kp_cmd_.get().get_value(); }
  double getKd()              const { return kd_cmd_.get().get_value(); }
  double getFeedforward()     const { return ff_cmd_.get().get_value(); }

  // ── Command writes ───────────────────────────────────────────────────────────
  void setPositionDesired(double cmd) { pos_cmd_.get().set_value(cmd); }
  void setVelocityDesired(double cmd) { vel_cmd_.get().set_value(cmd); }
  void setKp(double cmd)              { kp_cmd_.get().set_value(cmd); }
  void setKd(double cmd)              { kd_cmd_.get().set_value(cmd); }
  void setFeedforward(double cmd)     { ff_cmd_.get().set_value(cmd); }

  /**
   * Convenience: write all five fields in one call.
   * Matches the original ROS1 API:
   *   handle.setCommand(posDes, velDes, kp, kd, ff)
   */
  void setCommand(double pos_des, double vel_des, double kp, double kd, double ff)
  {
    setPositionDesired(pos_des);
    setVelocityDesired(vel_des);
    setKp(kp);
    setKd(kd);
    setFeedforward(ff);
  }

private:
  std::string name_;

  std::reference_wrapper<hardware_interface::LoanedStateInterface>   pos_state_;
  std::reference_wrapper<hardware_interface::LoanedStateInterface>   vel_state_;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> pos_cmd_;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> vel_cmd_;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> kp_cmd_;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> kd_cmd_;
  std::reference_wrapper<hardware_interface::LoanedCommandInterface> ff_cmd_;
};

}  // namespace legged
