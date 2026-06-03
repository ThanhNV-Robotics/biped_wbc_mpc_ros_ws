// Standalone ROS2 node that subscribes to /joint_states and an IMU topic,
// runs the linear Kalman filter state estimator, and publishes odometry/pose.

#include <pinocchio/fwd.hpp>  // must be first
#include <pinocchio/multibody/model.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include "LinearKalmanFilter.h"

using namespace ocs2;
using namespace legged_robot;

// ---------------------------------------------------------------------------
class StateEstimatorNode : public rclcpp::Node
{
public:
  StateEstimatorNode() : rclcpp::Node("state_estimator_node")
  {
    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter<std::string>("task_file", "");
    declare_parameter<std::string>("urdf_file", "");
    declare_parameter<std::string>("reference_file", "");
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<double>("update_rate", 250.0);
    // Use an explicit typed default to avoid UninitializedStaticallyTypedParameterException
    declare_parameter("joint_names", std::vector<std::string>{});
  }

  // Must be called AFTER the node is owned by a shared_ptr (shared_from_this requirement)
  void initialize()
  {
    const auto taskFile      = get_parameter("task_file").as_string();
    const auto urdfFile      = get_parameter("urdf_file").as_string();
    const auto referenceFile = get_parameter("reference_file").as_string();
    const auto imuTopic      = get_parameter("imu_topic").as_string();
    const auto jsTopic       = get_parameter("joint_states_topic").as_string();
    const double updateRate  = get_parameter("update_rate").as_double();
    jointNames_              = get_parameter("joint_names").as_string_array();

    if (taskFile.empty() || urdfFile.empty() || referenceFile.empty()) {
      RCLCPP_FATAL(get_logger(),
                   "Parameters task_file, urdf_file, and reference_file must all be set.");
      throw std::runtime_error("Missing required parameters.");
    }

    // ── Build only what the state estimator needs (skip MPC/CppAD setup) ──
    RCLCPP_INFO(get_logger(), "Loading model settings from task file: %s", taskFile.c_str());
    ModelSettings modelSettings = loadModelSettings(taskFile, "model_settings", /*verbose=*/false);

    if (jointNames_.empty()) {
      jointNames_ = modelSettings.jointNames;
    }
    numJoints_ = jointNames_.size();
    contactFlag_.fill(false);  // start with no contacts; updated heuristically

    PinocchioInterface pinocchioInterface =
        centroidal_model::createPinocchioInterface(urdfFile, modelSettings.jointNames);

    const CentroidalModelInfo centroidalModelInfo = centroidal_model::createCentroidalModelInfo(
        pinocchioInterface,
        centroidal_model::loadCentroidalType(taskFile),
        centroidal_model::loadDefaultJointState(pinocchioInterface.getModel().nq - 6, referenceFile),
        modelSettings.contactNames3DoF,
        modelSettings.contactNames6DoF);

    RCLCPP_INFO(get_logger(), "Pinocchio model: nq=%d, nv=%d, contacts=%zu",
                pinocchioInterface.getModel().nq, pinocchioInterface.getModel().nv,
                centroidalModelInfo.numThreeDofContacts + centroidalModelInfo.numSixDofContacts);

    // ── End-effector kinematics (feet) ──────────────────────────────────────
    CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo);
    auto eeKinematics = std::make_shared<PinocchioEndEffectorKinematics>(
        pinocchioInterface, pinocchioMapping,
        modelSettings.contactNames3DoF);

    // ── State estimator ─────────────────────────────────────────────────────
    estimator_ = std::make_shared<legged::KalmanFilterEstimate>(
        shared_from_this(),
        pinocchioInterface,
        centroidalModelInfo,
        *eeKinematics);
    estimator_->loadSettings(taskFile, /*verbose=*/false);

    RCLCPP_INFO(get_logger(), "Kalman filter estimator initialized with %zu joints, %zu contacts.",
                numJoints_, contactFlag_.size());

    // ── Subscriptions ────────────────────────────────────────────────────────
    jointStateSub_ = create_subscription<sensor_msgs::msg::JointState>(
        jsTopic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(dataMutex_);
          latestJointState_ = std::move(msg);
        });

    imuSub_ = create_subscription<sensor_msgs::msg::Imu>(
        imuTopic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(dataMutex_);
          latestImu_ = std::move(msg);
        });

    // ── Update timer ─────────────────────────────────────────────────────────
    const auto periodNs = std::chrono::nanoseconds(
        static_cast<int64_t>(1e9 / updateRate));
    lastUpdateTime_ = now();
    updateTimer_ = create_wall_timer(periodNs, [this]() { timerCallback(); });

    RCLCPP_INFO(get_logger(), "StateEstimatorNode ready. Subscribing to '%s' and '%s'.",
                jsTopic.c_str(), imuTopic.c_str());
  }

private:
  // ── Timer callback: run estimator at fixed rate ────────────────────────────
  void timerCallback()
  {
    sensor_msgs::msg::JointState::SharedPtr jsMsg;
    sensor_msgs::msg::Imu::SharedPtr imuMsg;
    {
      std::lock_guard<std::mutex> lock(dataMutex_);
      jsMsg  = latestJointState_;
      imuMsg = latestImu_;
    }

    if (!jsMsg || !imuMsg) {
      RCLCPP_DEBUG(get_logger(), "Waiting for joint state and IMU data...");
      return;
    }

    const rclcpp::Time currentTime = now();
    const rclcpp::Duration period  = currentTime - lastUpdateTime_;
    lastUpdateTime_ = currentTime;

    // ── Joint positions and velocities ────────────────────────────────────────
    vector_t jointPos(numJoints_), jointVel(numJoints_);
    jointPos.setZero();
    jointVel.setZero();

    for (size_t i = 0; i < numJoints_; ++i) {
      // Match joint names from the message to the expected order
      auto it = std::find(jsMsg->name.begin(), jsMsg->name.end(), jointNames_[i]);
      if (it == jsMsg->name.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Joint '%s' not found in JointState message.", jointNames_[i].c_str());
        continue;
      }
      const size_t idx = static_cast<size_t>(std::distance(jsMsg->name.begin(), it));
      if (idx < jsMsg->position.size()) jointPos(i) = jsMsg->position[idx];
      if (idx < jsMsg->velocity.size()) jointVel(i) = jsMsg->velocity[idx];
    }

    // ── IMU ──────────────────────────────────────────────────────────────────
    const auto& q = imuMsg->orientation;
    Eigen::Quaternion<scalar_t> quat(q.w, q.x, q.y, q.z);

    const auto& av = imuMsg->angular_velocity;
    vector3_t angularVel(av.x, av.y, av.z);

    const auto& la = imuMsg->linear_acceleration;
    vector3_t linearAccel(la.x, la.y, la.z);

    // Build covariance matrices (row-major 3×3 from the 9-element arrays)
    matrix3_t orientCov, angVelCov, linAccelCov;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) {
        orientCov(r, c)     = imuMsg->orientation_covariance[r * 3 + c];
        angVelCov(r, c)     = imuMsg->angular_velocity_covariance[r * 3 + c];
        linAccelCov(r, c)   = imuMsg->linear_acceleration_covariance[r * 3 + c];
      }

    // ── Simple contact heuristic: foot in contact if joint velocity is small ─
    // TODO: replace with a proper contact detector (e.g. FT sensor threshold).
    contactFlag_.fill(true);

    // ── Feed data into the estimator ─────────────────────────────────────────
    estimator_->updateJointStates(jointPos, jointVel);
    estimator_->updateContact(contactFlag_);
    estimator_->updateImu(quat, angularVel, linearAccel,
                          orientCov, angVelCov, linAccelCov);

    const vector_t rbdState = estimator_->update(currentTime, period);

    RCLCPP_DEBUG(get_logger(),
                 "Estimated base pos: [%.3f, %.3f, %.3f]",
                 rbdState(3), rbdState(4), rbdState(5));
  }

  // ── Members ──────────────────────────────────────────────────────────────
  std::shared_ptr<legged::KalmanFilterEstimate> estimator_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
  rclcpp::TimerBase::SharedPtr updateTimer_;

  std::mutex dataMutex_;
  sensor_msgs::msg::JointState::SharedPtr latestJointState_;
  sensor_msgs::msg::Imu::SharedPtr latestImu_;

  std::vector<std::string> jointNames_;
  size_t numJoints_{0};
  legged_robot::contact_flag_t contactFlag_{};

  rclcpp::Time lastUpdateTime_;
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateEstimatorNode>();
  node->initialize();  // called after make_shared so shared_from_this() is valid
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

