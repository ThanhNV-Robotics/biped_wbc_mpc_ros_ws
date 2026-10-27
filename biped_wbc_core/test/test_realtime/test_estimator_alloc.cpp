// Real-time check of the per-cycle estimation calls (WBCController::estimate): after one warm-up
// cycle they must not allocate heap memory. Every heap allocation of the process goes through
// malloc and friends, which this test replaces (glibc symbol interposition, also for the shared
// libraries: Eigen, std::vector / operator new, Pinocchio), so the count covers all of them.
//   colcon test --packages-select biped_wbc_core --ctest-args -R estimator_alloc
//   build/biped_wbc_core/test_estimator_alloc   (prints the count per call)

#include <gtest/gtest.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstddef>
#include <string>
#include <vector>

#include <tsid/robots/robot-wrapper.hpp>

#include "biped_wbc/common/data_type.hpp"
#include "biped_wbc/common/urdf_geometry.hpp"
#include "biped_wbc/state_estimation/StateEstimator.hpp"
#include "biped_wbc/state_estimation/contact_fusion.hpp"

// ---------------------------------------------------------------------------
// Allocation counter: counts while g_counting is true, otherwise only forwards to glibc
// ---------------------------------------------------------------------------
namespace
{
bool g_counting = false;
std::size_t g_allocations = 0;
}  // namespace

extern "C" {
void *__libc_malloc(std::size_t size);
void *__libc_calloc(std::size_t n, std::size_t size);
void *__libc_realloc(void *ptr, std::size_t size);
void *__libc_memalign(std::size_t alignment, std::size_t size);

void *malloc(std::size_t size)
{
  if (g_counting) ++g_allocations;
  return __libc_malloc(size);
}
void *calloc(std::size_t n, std::size_t size)
{
  if (g_counting) ++g_allocations;
  return __libc_calloc(n, size);
}
void *realloc(void *ptr, std::size_t size)
{
  if (g_counting) ++g_allocations;
  return __libc_realloc(ptr, size);
}
void *memalign(std::size_t alignment, std::size_t size)
{
  if (g_counting) ++g_allocations;
  return __libc_memalign(alignment, size);
}
void *aligned_alloc(std::size_t alignment, std::size_t size)
{
  if (g_counting) ++g_allocations;
  return __libc_memalign(alignment, size);
}
int posix_memalign(void **ptr, std::size_t alignment, std::size_t size)
{
  if (g_counting) ++g_allocations;
  *ptr = __libc_memalign(alignment, size);
  return *ptr ? 0 : ENOMEM;
}
}

// heap allocations made by f()
template <typename F>
std::size_t countAllocations(F &&f)
{
  g_allocations = 0;
  g_counting = true;
  f();
  g_counting = false;
  return g_allocations;
}

// ---------------------------------------------------------------------------
// Estimation of WBCController::estimate() on the v2 biped, standing with bent knees (so the foot
// force is observable and every code path runs), sensor values changing every cycle
// ---------------------------------------------------------------------------
class EstimatorAlloc : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // robot model, estimators and sole corners as in WBCController::on_configure
    const std::vector<std::string> package_dirs;
    this->robot_ = std::make_unique<tsid::robots::RobotWrapper>(BIPED_URDF, package_dirs,
                                                                pinocchio::JointModelFreeFlyer(), false);
    const double dt = 0.002;
    this->se_ = std::make_unique<StateEstimator>(*this->robot_, dt, this->feet_[0], this->feet_[1]);
    this->cf_ = std::make_unique<ContactFusion>(this->robot_->model(), this->feet_, dt, 15.0);
    for (std::size_t i = 0; i < 2; ++i)
    {
      Eigen::Matrix3Xd corners;
      ASSERT_TRUE(utils::soleCornersFromUrdf(BIPED_URDF, this->feet_[i], Eigen::Vector3d::UnitZ(), corners));
      this->cf_->setSoleCorners(i, corners);
    }

    // buffers as in WBCController::AllocateBuffers
    const int na = this->robot_->na();
    this->sensor_ = RobotSensor(na);
    this->q_ = Eigen::VectorXd::Zero(this->robot_->nq());
    this->v_ = Eigen::VectorXd::Zero(this->robot_->nv());
    this->contact_probability_ = Eigen::VectorXd::Ones(2);

    // knees bent 0.3 rad (hip and ankle pitch -0.15 each), knee torque of half the weight per leg
    for (const std::string side : {"left", "right"})
    {
      this->setJoint(side + "_hip_pitch_joint", -0.15, 0.0);
      this->setJoint(side + "_knee_pitch_joint", 0.3, -5.0);
      this->setJoint(side + "_ankle_pitch_joint", -0.15, -1.0);
    }
    this->sensor_.imu_sensor.imu_accel_L << 0.0, 0.0, 9.81;

    // start as in on_activate
    this->se_->reset(this->sensor_);
    this->cf_->reset();
  }

  // joint `name` (Pinocchio order in the sensor) to position q, torque tau
  void setJoint(const std::string &name, double q, double tau)
  {
    const int j = this->robot_->model().joints[this->robot_->model().getJointId(name)].idx_v() - 6;
    this->sensor_.actuator_state.qj(j) = q;
    this->sensor_.actuator_state.torquej(j) = tau;
  }

  // sensor of cycle k: small changes, so no call can skip its work
  void perturb(int k)
  {
    const double s = 1e-3 * ((k % 7) - 3);
    this->sensor_.actuator_state.dqj.setConstant(s);
    this->sensor_.imu_sensor.imu_gyro_L << s, -s, 0.5 * s;
    this->sensor_.imu_sensor.imu_accel_L << s, 0.0, 9.81 + s;
  }

  const std::vector<std::string> feet_{"left_ankle_pitch_link", "right_ankle_pitch_link"};
  const std::vector<bool> scheduled_contact_{true, true};
  const std::vector<double> contact_phase_{0.5, 0.5};

  std::unique_ptr<tsid::robots::RobotWrapper> robot_;
  std::unique_ptr<StateEstimator> se_;
  std::unique_ptr<ContactFusion> cf_;
  RobotSensor sensor_;
  Eigen::VectorXd q_, v_, contact_probability_;
};

TEST_F(EstimatorAlloc, NoHeapAllocationPerCycle)
{
  // the four calls of WBCController::estimate(), in its order
  const auto se_update = [&] { this->se_->update(this->sensor_, this->contact_probability_); };
  const auto se_state = [&] { this->se_->getPinocchioState(this->q_, this->v_); };
  const auto cf_torque = [&] {
    this->cf_->computeDisturbanceTorque(this->q_, this->v_, this->sensor_.actuator_state.torquej);
  };
  const auto cf_fuse = [&] {
    this->contact_probability_ = this->cf_->fuseContactProbability(this->scheduled_contact_, this->contact_phase_);
  };

  // warm-up cycle: first-call work (e.g. the observer seed) may allocate once
  se_update();
  se_state();
  cf_torque();
  cf_fuse();

  // 1000 cycles (2 s at 500 Hz), allocations summed per call
  std::size_t n_update = 0, n_state = 0, n_torque = 0, n_fuse = 0;
  for (int k = 0; k < 1000; ++k)
  {
    this->perturb(k);
    n_update += countAllocations(se_update);
    n_state += countAllocations(se_state);
    n_torque += countAllocations(cf_torque);
    n_fuse += countAllocations(cf_fuse);
  }

  EXPECT_EQ(n_update, 0u) << "StateEstimator::update";
  EXPECT_EQ(n_state, 0u) << "StateEstimator::getPinocchioState";
  EXPECT_EQ(n_torque, 0u) << "ContactFusion::computeDisturbanceTorque (incl. foot wrenches)";
  EXPECT_EQ(n_fuse, 0u) << "ContactFusion::fuseContactProbability";

  // the estimate stays sensible (the fixes must not change the result)
  EXPECT_TRUE(this->q_.allFinite() && this->v_.allFinite());
  EXPECT_GT(this->cf_->getFootWrench(0)(2), 20.0) << "bent knees: foot force observable";
  std::printf("final: base z %.6f vx %.6f | fz %.4f %.4f | P %.4f %.4f\n", this->q_(2), this->v_(0),
              this->cf_->getFootWrench(0)(2), this->cf_->getFootWrench(1)(2), this->contact_probability_(0),
              this->contact_probability_(1));
}

// ---------------------------------------------------------------------------
// Timing of the same calls (informative, no limit): mean / median / p99 / max per call over 20000
// cycles (40 s at 500 Hz), after 1000 warm-up cycles. Budget at 500 Hz: 2000 us per cycle for
// everything (estimation + planner + TSID + hardware I/O).
// ---------------------------------------------------------------------------
TEST_F(EstimatorAlloc, Timing)
{
  using Clock = std::chrono::steady_clock;
  const auto se_update = [&] { this->se_->update(this->sensor_, this->contact_probability_); };
  const auto se_state = [&] { this->se_->getPinocchioState(this->q_, this->v_); };
  const auto cf_torque = [&] {
    this->cf_->computeDisturbanceTorque(this->q_, this->v_, this->sensor_.actuator_state.torquej);
  };
  const auto cf_fuse = [&] {
    this->contact_probability_ = this->cf_->fuseContactProbability(this->scheduled_contact_, this->contact_phase_);
  };

  // duration of f() [us]
  const auto time_us = [](auto &&f) {
    const auto t0 = Clock::now();
    f();
    return std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
  };

  const int n_warmup = 1000, n = 20000;
  std::vector<double> t_update(n), t_state(n), t_torque(n), t_fuse(n), t_total(n);
  for (int k = -n_warmup; k < n; ++k)
  {
    this->perturb(k + n_warmup);
    const auto t0 = Clock::now();
    const double a = time_us(se_update), b = time_us(se_state), c = time_us(cf_torque), d = time_us(cf_fuse);
    const double total = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
    if (k < 0) continue;
    t_update[k] = a;
    t_state[k] = b;
    t_torque[k] = c;
    t_fuse[k] = d;
    t_total[k] = total;
  }

  // statistics of one call
  const auto report = [](const char *name, std::vector<double> t) {
    std::sort(t.begin(), t.end());
    double sum = 0.0;
    for (double x : t) sum += x;
    std::printf("  %-42s mean %7.2f  median %7.2f  p99 %7.2f  max %7.2f us\n", name, sum / t.size(),
                t[t.size() / 2], t[static_cast<std::size_t>(0.99 * t.size())], t.back());
  };
  std::printf("estimate() timing, %d cycles:\n", n);
  report("StateEstimator::update", t_update);
  report("StateEstimator::getPinocchioState", t_state);
  report("ContactFusion::computeDisturbanceTorque", t_torque);
  report("ContactFusion::fuseContactProbability", t_fuse);
  report("total estimate()", t_total);
}
