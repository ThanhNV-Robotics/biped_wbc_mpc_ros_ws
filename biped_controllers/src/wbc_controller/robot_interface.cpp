#include "wbc_controller/robot_interface.hpp"

#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace biped_controllers
{

RobotInterface::RobotInterface(const ControllerParams &params, const JointMap &joint_map)
: joints_(params.joints), torque_mode_(params.torque_mode), enabled_interface_(params.enabled_interface),
  joint_map_(joint_map)
{
  // IMU helper: the names of its 10 state interfaces come from imu_name (= <sensor name> in
  // biped_hw.urdf.xacro)
  this->imu_ = std::make_unique<semantic_components::IMUSensor>(params.imu_name);

  // command buffers with their final size: the real-time methods never allocate
  const std::size_t n = this->joints_.size();
  this->q_cmd_ = Eigen::VectorXd::Zero(n);
  this->v_cmd_ = Eigen::VectorXd::Zero(n);
  this->tau_cmd_ = Eigen::VectorXd::Zero(n);
}

std::vector<std::string> RobotInterface::commandInterfaceNames() const
{
  // PVT command of every joint; torque mode: the effort only (the drive then applies it alone)
  std::vector<std::string> names;
  for (const auto &joint : this->joints_)
  {
    if (!this->torque_mode_)
    {
      names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
      names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return names;
}

std::vector<std::string> RobotInterface::stateInterfaceNames() const
{
  // IMU: its 10 interfaces <imu_name>/orientation.x ... (names built by the helper)
  std::vector<std::string> names = this->imu_->get_state_interface_names();
  // joints: measured position, velocity and applied torque
  for (const auto &joint : this->joints_)
  {
    names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  // motors enabled (1) or off (0)
  names.push_back(this->enabled_interface_);
  return names;
}

bool RobotInterface::bind(std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces,
                          std::vector<hardware_interface::LoanedStateInterface> &state_interfaces, std::string &error)
{
  // IMU: bind the helper to its 10 loaned state interfaces; fails if the hardware has no sensor
  // <imu_name> (e.g. imu_name differs from the xacro)
  if (!this->imu_->assign_loaned_state_interfaces(state_interfaces))
  {
    error = "IMU state interfaces not found (imu_name must match the <sensor> in the xacro)";
    return false;
  }

  // the loan <joint>/<interface>, nullptr if missing
  const auto find = [](auto &loans, const std::string &joint, const char *interface) {
    for (auto &loan : loans)
    {
      if (loan.get_prefix_name() == joint && loan.get_interface_name() == interface)
      {
        return &loan;
      }
    }
    return static_cast<decltype(&loans[0])>(nullptr);
  };

  // joints, ros2_control order
  const std::size_t n = this->joints_.size();
  this->pos_cmd_.assign(n, nullptr);
  this->vel_cmd_.assign(n, nullptr);
  this->eff_cmd_.assign(n, nullptr);
  this->pos_state_.assign(n, nullptr);
  this->vel_state_.assign(n, nullptr);
  this->eff_state_.assign(n, nullptr);
  for (std::size_t i = 0; i < n; ++i)
  {
    const std::string &joint = this->joints_[i];
    this->pos_cmd_[i] = find(command_interfaces, joint, hardware_interface::HW_IF_POSITION);
    this->vel_cmd_[i] = find(command_interfaces, joint, hardware_interface::HW_IF_VELOCITY);
    this->eff_cmd_[i] = find(command_interfaces, joint, hardware_interface::HW_IF_EFFORT);
    this->pos_state_[i] = find(state_interfaces, joint, hardware_interface::HW_IF_POSITION);
    this->vel_state_[i] = find(state_interfaces, joint, hardware_interface::HW_IF_VELOCITY);
    this->eff_state_[i] = find(state_interfaces, joint, hardware_interface::HW_IF_EFFORT);
    const bool pv_cmd = this->torque_mode_ || (this->pos_cmd_[i] && this->vel_cmd_[i]);
    if (!pv_cmd || !this->eff_cmd_[i] || !this->pos_state_[i] || !this->vel_state_[i] || !this->eff_state_[i])
    {
      error = "missing interfaces for joint '" + joint + "'";
      this->release();
      return false;
    }
  }

  // motors-enabled state interface (full name, e.g. BipedSystem/enabled)
  this->enabled_state_ = nullptr;
  for (auto &loan : state_interfaces)
  {
    if (loan.get_name() == this->enabled_interface_)
    {
      this->enabled_state_ = &loan;
    }
  }
  if (!this->enabled_state_)
  {
    error = "missing state interface '" + this->enabled_interface_ + "'";
    this->release();
    return false;
  }
  return true;
}

void RobotInterface::release()
{
  // drop the references to the loans before the controller manager takes them back
  this->imu_->release_interfaces();
  this->pos_cmd_.clear();
  this->vel_cmd_.clear();
  this->eff_cmd_.clear();
  this->pos_state_.clear();
  this->vel_state_.clear();
  this->eff_state_.clear();
  this->enabled_state_ = nullptr;
}

bool RobotInterface::motorsEnabled() const
{
  return this->enabled_state_ && this->enabled_state_->get_value() > 0.5;
}

bool RobotInterface::read(RobotSensor &sensor)
{
  // joints: ros2_control index i -> Pinocchio index j
  auto &joints = sensor.actuator_state;
  for (std::size_t i = 0; i < this->joints_.size(); ++i)
  {
    const int j = this->joint_map_.toPin(i);
    joints.qj(j) = this->pos_state_[i]->get_value();
    joints.dqj(j) = this->vel_state_[i]->get_value();
    joints.torquej(j) = this->eff_state_[i]->get_value();
  }

  // IMU: orientation comes as {x, y, z, w}, Eigen's quaternion constructor takes (w, x, y, z);
  // gyro and accelerometer are in the IMU frame (copied to locals, std::array by value)
  auto &imu = sensor.imu_sensor;
  const auto o = this->imu_->get_orientation();
  const auto w = this->imu_->get_angular_velocity();
  const auto a = this->imu_->get_linear_acceleration();
  imu.imu_quat_ = Quat(o[3], o[0], o[1], o[2]);
  imu.imu_gyro_L << w[0], w[1], w[2];
  imu.imu_accel_L << a[0], a[1], a[2];

  // all values finite (a broken interface reads NaN)
  return joints.qj.allFinite() && joints.dqj.allFinite() && joints.torquej.allFinite() &&
         imu.imu_quat_.coeffs().allFinite() && imu.imu_gyro_L.allFinite() && imu.imu_accel_L.allFinite();
}

bool RobotInterface::holdCurrentCommand(std::string &error)
{
  for (std::size_t i = 0; i < this->joints_.size(); ++i)
  {
    const double q = this->pos_state_[i]->get_value();
    if (!std::isfinite(q))
    {
      error = "joint '" + this->joints_[i] + "' has no valid position";
      return false;
    }
    const double p = this->pos_cmd_[i] ? this->pos_cmd_[i]->get_value() : q;
    this->q_cmd_(i) = std::isfinite(p) ? p : q;
    this->v_cmd_(i) = 0.0;
    this->tau_cmd_(i) = 0.0;
  }
  // written now, so the drives get it before the first update()
  this->writeCommand();
  return true;
}

void RobotInterface::holdMeasured()
{
  for (std::size_t i = 0; i < this->joints_.size(); ++i)
  {
    const double q = this->pos_state_[i]->get_value();
    if (std::isfinite(q))  // keep the last position command if the reading is broken
      this->q_cmd_(i) = q;
    this->v_cmd_(i) = 0.0;
    this->tau_cmd_(i) = 0.0;
  }
  this->writeCommand();
}

void RobotInterface::write(const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &v,
                           const Eigen::Ref<const Eigen::VectorXd> &tau)
{
  // Pinocchio order -> ros2_control order
  this->joint_map_.toRosOrder(q, this->q_cmd_);
  this->joint_map_.toRosOrder(v, this->v_cmd_);
  this->joint_map_.toRosOrder(tau, this->tau_cmd_);
  this->writeCommand();
}

void RobotInterface::writeLastCommand()
{
  this->writeCommand();
}

void RobotInterface::writeCommand()
{
  for (std::size_t i = 0; i < this->joints_.size(); ++i)
  {
    if (!this->torque_mode_)
    {
      this->pos_cmd_[i]->set_value(this->q_cmd_(i));
      this->vel_cmd_[i]->set_value(this->v_cmd_(i));
    }
    this->eff_cmd_[i]->set_value(this->tau_cmd_(i));
  }
}

}  // namespace biped_controllers
