#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>

namespace biped_controllers
{

// Joint order maps between ros2_control and the Pinocchio model.
//   "ros2_control order" = order of the `joints` parameter (used for the interfaces)
//   "Pinocchio order"    = order of the actuated joints in the model = q[7:], v[6:], tau of TSID
//                          and the estimators
// Every conversion between the two goes through this map, so the order in controllers.yaml is free.
// No ROS dependency: errors are returned as a message.
class JointMap
{
public:
  // checks `joints` against the free-flyer model (one name per actuated joint, each an actuated
  // 1-DoF joint, no duplicates) and builds the maps; false + `error` set if a check fails
  bool build(const pinocchio::Model &model, const std::vector<std::string> &joints, std::string &error);

  std::size_t size() const { return this->ros_to_pin_.size(); }

  // index conversions: ros2_control index i <-> Pinocchio index j
  int toPin(std::size_t i) const { return this->ros_to_pin_[i]; }
  int toRos(std::size_t j) const { return this->pin_to_ros_[j]; }

  // vector conversions (size() entries each, no allocation: usable in update())
  void toPinOrder(const Eigen::Ref<const Eigen::VectorXd> &ros, Eigen::Ref<Eigen::VectorXd> pin) const;
  void toRosOrder(const Eigen::Ref<const Eigen::VectorXd> &pin, Eigen::Ref<Eigen::VectorXd> ros) const;

  // "  <name> -> <pinocchio index>" per line, to log the mapping once
  std::string describe(const std::vector<std::string> &joints) const;

private:
  std::vector<int> ros_to_pin_;  // ros_to_pin_[i] = Pinocchio index of joints[i]
  std::vector<int> pin_to_ros_;  // pin_to_ros_[j] = index in joints of the j-th actuated joint of the model
};

}  // namespace biped_controllers
