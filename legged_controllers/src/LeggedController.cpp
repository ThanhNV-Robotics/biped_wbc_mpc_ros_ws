//
// Created by qiayuan on 2022/6/24.
//

// pinocchio/fwd.hpp must be included before any other Pinocchio header to
// avoid multiple template instantiation errors.
#include <pinocchio/fwd.hpp>

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

namespace legged {

LeggedController::~LeggedController()
{
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "\n### MPC Benchmarking"
            << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << " [ms]"
            << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << " [ms]\n"
            << "\n### WBC Benchmarking"
            << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << " [ms]"
            << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << " [ms]\n";
}

controller_interface::InterfaceConfiguration LeggedController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // position + velocity + feedforward_effort for each actuated joint.
  // MuJoCo exports feedforward_effort for all actuators and activates PD impedance
  // mode when both position/velocity AND feedforward_effort are claimed together.
  // Claiming raw "effort" would override PD control with zero torque (robot falls).
  for (const auto& joint : jointNames_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
    config.names.push_back(joint + "/feedforward_effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration LeggedController::state_interface_configuration() const
{
  // config the state interface
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // joint position + velocity for state estimation; effort to read actual torques
  for (const auto& joint : jointNames_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
    config.names.push_back(joint + "/effort");
  }

  // IMU: 4 orientation + 3 angular velocity + 3 linear acceleration fields
  for (const auto* field : {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
                             "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
                             "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}) {
    config.names.push_back(imuName_ + "/" + field);
  }

  return config;
}

controller_interface::CallbackReturn LeggedController::on_init()
{
  // ── File path parameters (required) ─────────────────────────────────────
  urdfFile_      = get_node()->declare_parameter<std::string>("urdf_file", "");
  taskFile_      = get_node()->declare_parameter<std::string>("task_file", "");
  referenceFile_ = get_node()->declare_parameter<std::string>("reference_file", "");

  if (urdfFile_.empty() || taskFile_.empty() || referenceFile_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "urdf_file, task_file, and reference_file parameters must be set");
    return controller_interface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_node()->get_logger(), "URDF file: %s",      urdfFile_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Task file: %s",      taskFile_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Reference file: %s", referenceFile_.c_str());

  // ── Hardware interface parameters ────────────────────────────────────────
  // Joint names must match the ros2_control <joint> tags in the URDF exactly.
  jointNames_ = get_node()->declare_parameter<std::vector<std::string>>(
      "joints",
      {"left_hip_pitch_joint",  "left_hip_roll_joint",  "left_hip_yaw_joint",
       "left_knee_pitch_joint", "left_ankle_roll_joint", "left_ankle_pitch_joint",
       "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
       "right_knee_pitch_joint", "right_ankle_roll_joint", "right_ankle_pitch_joint"});

  // IMU sensor name must match the ros2_control <sensor> tag in the URDF.
  imuName_ = get_node()->declare_parameter<std::string>("imu_name", "torso_imu");

  RCLCPP_INFO(get_node()->get_logger(), "Controlling %zu joints, IMU: %s",
              jointNames_.size(), imuName_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

// ── Setup helpers ────────────────────────────────────────────────────────────

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                             const std::string& referenceFile, bool verbose)
{
  legged_interface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  legged_interface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc()
{
  using namespace ocs2;
  using namespace legged_robot;

  mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(),
                                   legged_interface_->sqpSettings(),
                                   legged_interface_->getOptimalControlProblem(),
                                   legged_interface_->getInitializer());

  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(
      legged_interface_->getPinocchioInterface(),
      legged_interface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";

  // GaitReceiver: subscribes to gait commands and updates the gait schedule
  // before each MPC solve. Requires a plain rclcpp::Node (not LifecycleNode).
  auto gaitReceiverPtr = std::make_shared<GaitReceiver>(
      mpcNode_,
      legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(),
      robotName);

  // RosReferenceManager: subscribes to target trajectory / mode schedule topics
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(
      robotName, legged_interface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(mpcNode_);

  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  observationPublisher_ = mpcNode_->create_publisher<ocs2_msgs::msg::MpcObservation>(
      robotName + "_mpc_observation", 1);
}

void LeggedController::setupMrt()
{
  mpcMrtInterface_ = std::make_shared<ocs2::MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&legged_interface_->getRollout());
  mpcTimer_.reset();
  // MPC thread is started in on_activate(), not here, so it can be
  // cleanly stopped and restarted across deactivate/activate cycles.
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

controller_interface::CallbackReturn LeggedController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Read verbose flag from the task file before any OCS2 object is created
  bool verbose = false;
  ocs2::loadData::loadCppDataType(taskFile_, "legged_robot_interface.verbose", verbose);

  // Create a plain rclcpp::Node for all OCS2 ROS pub/sub (GaitReceiver,
  // RosReferenceManager, observation publisher). The controller_manager gives
  // us a LifecycleNode via get_node(), which is incompatible with
  // rclcpp::Node::SharedPtr expected by the OCS2 ROS interfaces.
  mpcNode_ = std::make_shared<rclcpp::Node>("legged_mpc");

  // 3a: Build OCS2 optimal control problem from URDF + task files
  // compile CppAd lib if needed
  setupLeggedInterface(taskFile_, urdfFile_, referenceFile_, verbose);

  // 3a cont: End-effector kinematics — used by state estimator and WBC
  ocs2::CentroidalModelPinocchioMapping pinocchioMapping(legged_interface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<ocs2::PinocchioEndEffectorKinematics>(
      legged_interface_->getPinocchioInterface(), pinocchioMapping,
      legged_interface_->modelSettings().contactNames3DoF);

  // 3b: SQP MPC solver + ROS communication modules
  setupMpc();

  // 3c: MPC-MRT bridge (polling interface between MPC thread and update loop)
  setupMrt();

  // 3d: Kalman filter state estimator
  setupStateEstimate(taskFile_, verbose);

  // 3e: Load standing joint angles for PD position hold.
  // centroidal_model::loadDefaultJointState is the same function used internally
  // by LeggedInterface and correctly handles the (i,0) sparse format with comments.
  {
    const size_t numJoints = jointNames_.size();
    defaultJointAngles_ = ocs2::centroidal_model::loadDefaultJointState(numJoints, referenceFile_);
    RCLCPP_INFO(get_node()->get_logger(), "Loaded defaultJointState (%zu joints): [%.3f ... %.3f]",
                defaultJointAngles_.size(), defaultJointAngles_(0),
                defaultJointAngles_(defaultJointAngles_.size() - 1));
  }

  // 3g: WBC + SafetyChecker — TODO: enable once legged_wbc is ported to ROS2
  // wbc_ = std::make_shared<WeightedWbc>(legged_interface_->getPinocchioInterface(),
  //                                      legged_interface_->getCentroidalModelInfo(),
  //                                      *eeKinematicsPtr_);
  // wbc_->loadTasksSetting(taskFile_, verbose);
  // safetyChecker_ = std::make_shared<SafetyChecker>(legged_interface_->getCentroidalModelInfo());

  RCLCPP_INFO(get_node()->get_logger(), "LeggedController configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  using namespace ocs2;

  try {

  // ── 1. Initialise observation from current hardware state ─────────────────
  currentObservation_.state.setZero(legged_interface_->getCentroidalModelInfo().stateDim);
  currentObservation_.input.setZero(legged_interface_->getCentroidalModelInfo().inputDim);
  currentObservation_.time = 0.0;

  // Read the first hardware snapshot (2 ms dummy period — just to seed the filter).
  RCLCPP_INFO(get_node()->get_logger(), "on_activate: seeding state from hardware...");
  updateStatesEstimatate(get_node()->now(), rclcpp::Duration(0, 2'000'000));
  RCLCPP_INFO(get_node()->get_logger(), "on_activate: state seeded.");

  // ── 2. Send hold-in-place reference to the MPC ───────────────────────────
  TargetTrajectories initTarget({currentObservation_.time},
                                {currentObservation_.state},
                                {currentObservation_.input});
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(initTarget);
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // ── 3. Start the MPC background thread ──────────────────────────────────────
  mpcRunning_ = true;
  controllerRunning_ = true;
  mpcThread_ = std::thread([this]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [this]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            legged_interface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        RCLCPP_ERROR(get_node()->get_logger(), "[MPC thread] %s", e.what());
      }
    }
  });
  setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpcThread_);
  RCLCPP_INFO(get_node()->get_logger(), "STEP 1: MPC disabled — position-hold only. No advanceMpc() calls.");

  // ── 6. Seed standing-pose commands before first update() tick ───────────────
  // Command the standing joint angles (not current measured angles) so the PD
  // controller has a non-zero error to work against from the very first step.
  const size_t nJoints = jointNames_.size();
  for (size_t i = 0; i < nJoints; ++i) {
    command_interfaces_[3 * i + 0].set_value(defaultJointAngles_(i));
    command_interfaces_[3 * i + 1].set_value(0.0);
    command_interfaces_[3 * i + 2].set_value(0.0);
  }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_activate() threw: %s", e.what());
    controllerRunning_ = false;
    mpcRunning_ = false;
    if (mpcThread_.joinable()) mpcThread_.join();
    return controller_interface::CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  mpcRunning_ = false;
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeggedController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  using namespace ocs2;

  // ── 1. State estimation ───────────────────────────────────────────────────
  updateStatesEstimatate(time, period);

  // ── 2. Synchronise current observation with the MPC ──────────────────────
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // ── 3. Poll latest MPC policy (non-blocking, keeps the background solve in sync) ──
  mpcMrtInterface_->updatePolicy();

  // ── 4. Write standing-pose PD commands ───────────────────────────────────
  // Hold joints at the nominal standing configuration. The MuJoCo PD actuator
  // computes: τ = kp*(q_ref - q) + kd*(0 - qdot) + ff.
  // Commanding the current measured position gives zero error → zero torque;
  // commanding the standing angles gives a restoring force toward the pose.
  // TODO: replace with evaluatePolicy() + WBC once standing is verified.
  const size_t nJoints = jointNames_.size();
  for (size_t j = 0; j < nJoints; ++j) {
    command_interfaces_[3 * j + 0].set_value(defaultJointAngles_(j));
    command_interfaces_[3 * j + 1].set_value(0.0);
    command_interfaces_[3 * j + 2].set_value(0.0);
  }

  // ── 6. Publish observation for monitoring / command interface ─────────────
  observationPublisher_->publish(
      ocs2::ros_msg_conversions::createObservationMsg(currentObservation_));

  return controller_interface::return_type::OK;
}

void LeggedController::updateStatesEstimatate(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  using namespace ocs2;

  const size_t nJoints = jointNames_.size();

  // ── Joint encoders ────────────────────────────────────────────────────────
  // state_interfaces_ layout: [pos, vel, effort] per joint, then IMU fields
  vector_t jointPos(nJoints), jointVel(nJoints);
  for (size_t i = 0; i < nJoints; ++i) {
    jointPos[i] = state_interfaces_[3 * i + 0].get_value();
    jointVel[i] = state_interfaces_[3 * i + 1].get_value();
  }
  stateEstimate_->updateJointStates(jointPos, jointVel);

  // ── IMU ───────────────────────────────────────────────────────────────────
  // state_interfaces_ layout after joints: orientation.{x,y,z,w}, ang_vel.{x,y,z}, lin_accel.{x,y,z}
  const size_t imuOff = 3 * nJoints;
  Eigen::Quaternion<scalar_t> quat(
      state_interfaces_[imuOff + 3].get_value(),   // w
      state_interfaces_[imuOff + 0].get_value(),   // x
      state_interfaces_[imuOff + 1].get_value(),   // y
      state_interfaces_[imuOff + 2].get_value());  // z
  vector3_t angVelLocal(
      state_interfaces_[imuOff + 4].get_value(),
      state_interfaces_[imuOff + 5].get_value(),
      state_interfaces_[imuOff + 6].get_value());
  vector3_t linAccelLocal(
      state_interfaces_[imuOff + 7].get_value(),
      state_interfaces_[imuOff + 8].get_value(),
      state_interfaces_[imuOff + 9].get_value());
  // Pass zero covariances — the KalmanFilter uses its own tuned Q/R matrices.
  stateEstimate_->updateImu(quat, angVelLocal, linAccelLocal,
                            matrix3_t::Zero(), matrix3_t::Zero(), matrix3_t::Zero());

  // ── Contact flags ─────────────────────────────────────────────────────────
  // Both feet always assumed in contact until force-sensor contact detection is wired up.
  legged_robot::contact_flag_t contactFlag;
  contactFlag.fill(false);
  const size_t numContacts = legged_interface_->getCentroidalModelInfo().numThreeDofContacts +
                             legged_interface_->getCentroidalModelInfo().numSixDofContacts;
  for (size_t i = 0; i < numContacts && i < contactFlag.size(); ++i) {
    contactFlag[i] = true;
  }
  stateEstimate_->updateContact(contactFlag);

  // ── Run Kalman filter → full rigid-body state ─────────────────────────────
  measuredRbdState_ = stateEstimate_->update(time, period);

  // ── Convert to centroidal state for MPC ──────────────────────────────────
  currentObservation_.time += period.seconds();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  // Unwrap yaw using std::remainder to handle wrap-around at ±π
  scalar_t yawNew = currentObservation_.state(9);
  currentObservation_.state(9) = yawLast + std::remainder(yawNew - yawLast, 2.0 * M_PI);
  currentObservation_.mode = stateEstimate_->getMode();
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose)
{
  // KalmanFilterEstimate needs its own plain rclcpp::Node for publishing /odom
  // and /pose — same reason mpcNode_ exists (LifecycleNode is incompatible).
  auto estimatorNode = std::make_shared<rclcpp::Node>("legged_state_estimator");
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(estimatorNode,
                                                          legged_interface_->getPinocchioInterface(),
                                                          legged_interface_->getCentroidalModelInfo(),
                                                          *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

}// namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerInterface)
