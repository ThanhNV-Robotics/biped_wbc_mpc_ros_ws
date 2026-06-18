//
// Created by qiayuan on 2022/6/24.
//

// pinocchio/fwd.hpp must be included before any other Pinocchio header to
// avoid multiple template instantiation errors.
#include <pinocchio/fwd.hpp>

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
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

  // position + velocity + effort command for each actuated joint.
  // Matches the URDF <command_interface> tags exported when use_pid:=true.
  for (const auto& joint : jointNames_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
    config.names.push_back(joint + "/effort");
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

  // 3e: WBC + SafetyChecker — TODO: enable once legged_wbc is ported to ROS2
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
  // TODO(step3): initialize observation, wait for initial MPC policy, start MPC thread
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // TODO(step3): stop MPC thread, release interfaces
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeggedController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // TODO(step3): state estimation → MPC query → WBC → safety check → write commands
  return controller_interface::return_type::OK;
}

void LeggedController::updateStatesEstimatate(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // TODO(step3): update state estimation (e.g., using a complementary filter or a Kalman filter)
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose)
{
  // TODO(step3): set up state estimation (e.g., subscribe to IMU and joint state topics)
  // This function ready to be called in taskfile.info which include the Kalman filter parameters and initilize the robot state estimation

  // step 1: init the Kalman filter state estimation
  // The estimator gets its own rclcpp::Node (not the lifecycle node) so it can
  // publish odom and subscribe independently of the controller lifecycle state.
  auto estimatorNode = std::make_shared<rclcpp::Node>("legged_state_estimator");
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(estimatorNode,
                                                          legged_interface_->getPinocchioInterface(),
                                                          legged_interface_->getCentroidalModelInfo(),
                                                          *eeKinematicsPtr_);
  // step 2: load the Kalman filter parameters from the task file
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

}// namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerInterface)
