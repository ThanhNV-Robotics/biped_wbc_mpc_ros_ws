#include "mujoco_ros2_control/biped_pvt_drive.hpp"

#include <cmath>
#include <cstdio>

namespace mujoco_ros2_control
{

namespace
{
// A limp joint hanging in its end stop sits slightly beyond the range (soft MuJoCo limits)
constexpr double kRangeToleranceRad = 0.01;
}  // namespace

std::string BipedPvtDrive::configure(const mjModel* m, std::vector<JointConfig> joints, const Params& params)
{
  for (const auto& j : joints)
  {
    if (j.qpos_adr < 0 || j.qvel_adr < 0 || j.actuator_id < 0 || j.actuator_id >= m->nu)
    {
      return "joint '" + j.name + "' has no hinge joint with a motor actuator in the MJCF";
    }
    // ctrl is used as the joint torque: a motor actuator acting on the joint with gear 1
    const int a = j.actuator_id;
    if (m->actuator_trntype[a] != mjTRN_JOINT || m->actuator_gaintype[a] != mjGAIN_FIXED ||
        m->actuator_biastype[a] != mjBIAS_NONE || m->actuator_gainprm[a * mjNGAIN] != 1.0 ||
        m->actuator_gear[a * 6] != 1.0)
    {
      return "actuator of joint '" + j.name + "' must be <motor joint=\"...\"/> with gear 1";
    }
  }

  joints_ = std::move(joints);
  params_ = params;
  const std::size_t n = joints_.size();
  for (Setpoint* s : { &cmd_, &sp_ })
  {
    s->pos.assign(n, 0.0);
    s->vel.assign(n, 0.0);
    s->eff.assign(n, 0.0);
  }
  state_ = State::DISABLED;
  return "";
}

void BipedPvtDrive::set_commands(const std::vector<double>& pos, const std::vector<double>& vel,
                                 const std::vector<double>& eff, double sim_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  bool finite = true;
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    cmd_.pos[i] = pos[i];
    cmd_.vel[i] = vel[i];
    cmd_.eff[i] = eff[i];
    finite = finite && std::isfinite(pos[i]) && std::isfinite(vel[i]) && std::isfinite(eff[i]);
  }
  cmd_.finite = finite;
  cmd_.time = sim_time;
}

std::string BipedPvtDrive::activation_blocker(const std::vector<double>& q) const
{
  std::string why;
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    const auto& j = joints_[i];
    if (!std::isfinite(q[i]))
    {
      why += " " + j.name + " has no valid position;";
    }
    else if (j.limited && (q[i] < j.lower - kRangeToleranceRad || q[i] > j.upper + kRangeToleranceRad))
    {
      char buf[160];
      std::snprintf(buf, sizeof(buf), " %s at %.3f rad, outside [%.3f, %.3f];", j.name.c_str(), q[i], j.lower,
                    j.upper);
      why += buf;
    }
  }
  return why;
}

void BipedPvtDrive::enable(const std::vector<double>& q, double sim_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    cmd_.pos[i] = q[i];
    cmd_.vel[i] = 0.0;
    cmd_.eff[i] = 0.0;
  }
  cmd_.finite = true;
  cmd_.time = sim_time;
  fault_.clear();
  state_ = State::ENABLED;
}

void BipedPvtDrive::disable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = State::DISABLED;
}

std::string BipedPvtDrive::fault() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fault_;
}

void BipedPvtDrive::apply(mjData* d)
{
  State state;
  {
    // State changes happen under the lock, so a disable() from the control thread is never undone
    std::lock_guard<std::mutex> lock(mutex_);
    sp_.pos = cmd_.pos;  // same size: no allocation
    sp_.vel = cmd_.vel;
    sp_.eff = cmd_.eff;
    sp_.time = cmd_.time;
    sp_.finite = cmd_.finite;

    state = state_.load();
    if (state == State::ENABLED)
    {
      // age < 0 after a sim reset (time jumped back): the next command refreshes it
      const double age = d->time - sp_.time;
      const bool timeout = params_.command_timeout > 0.0 && age > params_.command_timeout;
      if (timeout || !sp_.finite)
      {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s: damping (kd %.2f) for %.2f s, then disabled",
                      sp_.finite ? "command timeout" : "non-finite command", params_.damping_kd,
                      params_.damping_duration);
        fault_ = buf;
        damping_start_ = d->time;
        state = State::DAMPING;
      }
    }
    if (state == State::DAMPING && !(d->time - damping_start_ < params_.damping_duration))
    {
      state = State::DISABLED;
    }
    state_ = state;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    const auto& j = joints_[i];
    const double q = d->qpos[j.qpos_adr];
    const double dq = d->qvel[j.qvel_adr];
    double tau = 0.0;
    if (state == State::ENABLED)
    {
      tau = j.kp * (sp_.pos[i] - q) + j.kd * (sp_.vel[i] - dq) + sp_.eff[i];
    }
    else if (state == State::DAMPING)
    {
      tau = -params_.damping_kd * dq;
    }
    d->ctrl[j.actuator_id] = tau;  // MuJoCo clamps to the actuator ctrlrange (motor torque limit)
  }
}

}  // namespace mujoco_ros2_control
