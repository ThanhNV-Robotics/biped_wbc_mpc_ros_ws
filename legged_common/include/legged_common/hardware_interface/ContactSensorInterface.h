//
// ROS2 port of legged_common/hardware_interface/ContactSensorInterface.h
// Original by qiayuan (2021/11/5).
//
// ROS1 used HardwareResourceManager<ContactSensorHandle> as the interface registry.
// In ROS2 ros2_control that registry is replaced by the combination of
//   - SystemInterface::export_state_interfaces()  (hardware side — exports "<name>/contact")
//   - ControllerInterface::state_interface_configuration()  (controller side)
//
// This file provides ContactSensorHandle — a read-only wrapper around the single
// LoanedStateInterface that carries the foot-contact boolean (1.0 = in contact).
// The controller creates one handle per foot in on_activate() and calls
// isContact() inside update(), exactly as in ROS1.
//
#pragma once

#include <functional>
#include <string>

#include <hardware_interface/loaned_state_interface.hpp>

namespace legged {

/**
 * Read-only handle for a single foot-contact state interface.
 *
 * Required state interface (exported by MujocoSystemInterface):
 *   <sensor_name>/contact   (1.0 = touching ground, 0.0 = in the air)
 *
 * Declared in the URDF ros2_control block:
 *   <sensor name="<NAME>">
 *     <state_interface name="contact"/>
 *     <param name="mujoco_type">contact</param>
 *     <param name="mujoco_body_name"><BODY_NAME></param>
 *   </sensor>
 */
class ContactSensorHandle
{
public:
  ContactSensorHandle() = delete;

  ContactSensorHandle(const std::string& name,
                      hardware_interface::LoanedStateInterface& contact_state)
    : name_(name), contact_state_(contact_state)
  {}

  const std::string& getName() const { return name_; }

  /** Returns true when the foot is in contact with the ground. */
  bool isContact() const { return contact_state_.get().get_value() >= 0.5; }

private:
  std::string name_;
  std::reference_wrapper<hardware_interface::LoanedStateInterface> contact_state_;
};

}  // namespace legged
