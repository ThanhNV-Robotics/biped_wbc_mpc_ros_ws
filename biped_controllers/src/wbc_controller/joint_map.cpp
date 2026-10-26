#include "wbc_controller/joint_map.hpp"

namespace biped_controllers
{

bool JointMap::build(const pinocchio::Model &model, const std::vector<std::string> &joints, std::string &error)
{
  // actuated joints of a free-flyer model: all velocities but the 6 of the base
  const int na = model.nv - 6;

  // one name per actuated joint of the model
  if (static_cast<int>(joints.size()) != na)
  {
    error = "'joints' has " + std::to_string(joints.size()) + " names, the model has " + std::to_string(na) +
            " actuated joints (set 'joints' in controllers.yaml)";
    return false;
  }

  this->ros_to_pin_.assign(na, -1);
  this->pin_to_ros_.assign(na, -1);
  for (int i = 0; i < na; ++i)
  {
    const std::string &name = joints[i];

    // the joint must exist in the URDF model (getJointId returns model.njoints if not)
    const pinocchio::JointIndex jid = model.getJointId(name);
    if (jid >= static_cast<pinocchio::JointIndex>(model.njoints))
    {
      error = "joint '" + name + "' is not in the robot model";
      return false;
    }

    // a 1-DoF joint after the free flyer: its place among the actuated joints is its velocity
    // index minus the 6 base velocities (joint 0 = universe, 1 = free flyer)
    if (model.joints[jid].nv() != 1 || model.joints[jid].idx_v() < 6)
    {
      error = "joint '" + name + "' is not an actuated 1-DoF joint";
      return false;
    }
    const int j = model.joints[jid].idx_v() - 6;

    // each joint only once
    if (this->pin_to_ros_[j] != -1)
    {
      error = "joint '" + name + "' is listed twice in 'joints'";
      return false;
    }
    this->ros_to_pin_[i] = j;
    this->pin_to_ros_[j] = i;
  }
  return true;
}

void JointMap::toPinOrder(const Eigen::Ref<const Eigen::VectorXd> &ros, Eigen::Ref<Eigen::VectorXd> pin) const
{
  for (std::size_t i = 0; i < this->ros_to_pin_.size(); ++i)
  {
    pin(this->ros_to_pin_[i]) = ros(i);
  }
}

void JointMap::toRosOrder(const Eigen::Ref<const Eigen::VectorXd> &pin, Eigen::Ref<Eigen::VectorXd> ros) const
{
  for (std::size_t j = 0; j < this->pin_to_ros_.size(); ++j)
  {
    ros(this->pin_to_ros_[j]) = pin(j);
  }
}

std::string JointMap::describe(const std::vector<std::string> &joints) const
{
  std::string mapping;
  for (std::size_t i = 0; i < this->ros_to_pin_.size() && i < joints.size(); ++i)
  {
    mapping += "\n  " + joints[i] + " -> " + std::to_string(this->ros_to_pin_[i]);
  }
  return mapping;
}

}  // namespace biped_controllers
