#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/imu_sensor.hpp"

#include "biped_wbc/common/data_type.hpp"
#include "wbc_controller/controller_params.hpp"
#include "wbc_controller/joint_map.hpp"

namespace biped_controllers
{

// Everything that touches the ros2_control interfaces of the biped: which interfaces to claim,
// finding the loaned ones by name, reading joints + IMU into a RobotSensor, writing the PVT
// commands, and the hold commands. The controller only sees Pinocchio-order vectors.
//
// Lifecycle (called by the controller):
//   on_configure        RobotInterface(params, joint_map)
//   *_configuration()   commandInterfaceNames() / stateInterfaceNames()
//   on_activate         bind(...) -> motorsEnabled() -> holdCurrentCommand(...)
//   update()            read(sensor) ... write(q, v, tau) / writeLastCommand() / holdMeasured()
//   on_deactivate       release() (the last command stays in the interfaces: the next controller
//                       takes it over with its own bumpless hold)
class RobotInterface
{
public:
  RobotInterface(const ControllerParams &params, const JointMap &joint_map);

  // interfaces to claim: <joint>/position, velocity, effort (command and state), the IMU's 10 state
  // interfaces and the motors-enabled state interface
  std::vector<std::string> commandInterfaceNames() const;
  std::vector<std::string> stateInterfaceNames() const;

  // on_activate: finds every interface in the controller's loans by name (the loan order is not
  // guaranteed) and binds the IMU helper; false + `error` set if one is missing. The loans must
  // stay valid until release() (they do while the controller is active).
  bool bind(std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces,
            std::vector<hardware_interface::LoanedStateInterface> &state_interfaces, std::string &error);

  // on_deactivate: drops every reference to the loans
  void release();

  // motors enabled (enabled state interface = 1)
  bool motorsEnabled() const;

  // joints (Pinocchio order) + IMU -> sensor; false if a value is not finite. Real-time.
  bool read(RobotSensor &sensor);

  // on_activate, bumpless: keeps the position command the drives already hold (the hardware's own
  // hold or the previous controller's last command), measured q if that command is NaN; v = tau = 0.
  // Under gravity the PD hold lags its command, and that lag is the torque holding the robot up:
  // commanding p = measured q would drop it and the robot would sag. false + `error` if a
  // joint has no valid position.
  bool holdCurrentCommand(std::string &error);

  // hold at the measured position (q = measured, v = tau = 0) and write it: while the motors are
  // disabled, so there is no jump once they are enabled again. Real-time.
  void holdMeasured();

  // command in Pinocchio order (na each): position, velocity, feedforward torque. Real-time.
  void write(const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &v,
             const Eigen::Ref<const Eigen::VectorXd> &tau);

  // write the last command again: every cycle needs a fresh command (hardware command timeout),
  // also when nothing changes or the reading was invalid. Real-time.
  void writeLastCommand();

private:
  std::vector<std::string> joints_;  // ros2_control names, ros2_control order
  std::string enabled_interface_;    // e.g. BipedSystem/enabled
  JointMap joint_map_;

  // IMU helper (10 state interfaces <imu_name>/orientation.x ... /linear_acceleration.z)
  std::unique_ptr<semantic_components::IMUSensor> imu_;

  // loaned interfaces of each joint, ros2_control order; set by bind(), cleared by release()
  std::vector<hardware_interface::LoanedCommandInterface *> pos_cmd_, vel_cmd_, eff_cmd_;
  std::vector<hardware_interface::LoanedStateInterface *> pos_state_, vel_state_, eff_state_;
  hardware_interface::LoanedStateInterface *enabled_state_ = nullptr;

  // last command, ros2_control order (what writeLastCommand() sends)
  Eigen::VectorXd q_cmd_, v_cmd_, tau_cmd_;

  void writeCommand();  // q_cmd_ / v_cmd_ / tau_cmd_ -> command interfaces
};

}  // namespace biped_controllers
