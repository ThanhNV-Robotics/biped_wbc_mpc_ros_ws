#include <ocs2_sqp/SqpMpc.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <mujoco_sim_interface/MujocoRos2ControlInterface.h> //Use MujocoRos2ControlInterface to interact with mujoco_ros2_control sim


#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_centroidal_mpc/mrt/CentroidalMpcMrtJointController.h>
#include "humanoid_common_mpc_ros2/ros_comm/Ros2ProceduralMpcMotionManager.h"
#include "humanoid_common_mpc_ros2/visualization/HumanoidVisualizer.h"

using namespace ocs2;
using namespace ocs2::humanoid;
using namespace std;

sensor_msgs::msg::Imu::SharedPtr Imu_msg;
sensor_msgs::msg::JointState::SharedPtr JointState_msg;


// callback for joint state subscriber
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  JointState_msg = msg;
}

// callback for imu subsciber
void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{  
  Imu_msg = msg;
}

int main(int argc, char** argv) {

  std::vector<std::string> programArgs;
  programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() < 6) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }

  const std::string robotName(argv[1]);
  const std::string taskFile(argv[2]);
  const std::string referenceFile(argv[3]);
  const std::string urdfFile(argv[4]);
  const std::string gaitFile(argv[5]);
  const std::string mjxFile(argv[6]);

  rclcpp::init(argc, argv);

  // Robot interface
  CentroidalMpcInterface interface(taskFile, urdfFile, referenceFile);

  // MPC
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());

  // Launch MPC ROS node
  rclcpp::Node::SharedPtr nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_centroidal_mpc");

  auto qos = rclcpp::QoS(1);
  qos.best_effort();

  // Create a subscription to subsribe to /joint_states
  auto subscription_joint_states = nodeHandle->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, joint_state_callback );

  // create a subscription to subscribe to /imu
  auto subscription_imu = nodeHandle->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_sensor_broadcaster/imu", 10, imu_callback );
  
  // create a publisher to publish joint state from robotInterface.getRobotState() to /joint_states
  auto publisher_joint_states = nodeHandle->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);


  std::shared_ptr<HumanoidVisualizer> humanoidVisualizer(
      new HumanoidVisualizer(taskFile, interface.getPinocchioInterface(), interface.getMpcRobotModel(), nodeHandle));

  // Reference and motion management for Procedural MPC
  CentroidalMpcTargetTrajectoriesCalculator mpcTargetTrajectoriesCalculator(
      referenceFile, interface.getMpcRobotModel(), interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
      interface.mpcSettings().timeHorizon_);
  ProceduralMpcMotionManager::VelocityTargetToTargetTrajectories targetTrajectoriesFunc =
      [&mpcTargetTrajectoriesCalculator](const vector4_t& velocityTarget, scalar_t initTime, scalar_t finalTime,
                                         const vector_t& initState) mutable {
        return mpcTargetTrajectoriesCalculator.commandedVelocityToTargetTrajectories(velocityTarget, initTime, initState);
      };
  auto ros2ProceduralMpcMotionManager = std::make_shared<Ros2ProceduralMpcMotionManager>(
      gaitFile, referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(), targetTrajectoriesFunc);

  ros2ProceduralMpcMotionManager->subscribe(nodeHandle, qos);

  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  mpc.getSolverPtr()->addSynchronizedModule(ros2ProceduralMpcMotionManager);

  // Init Sim state
  // this is from robot_model package
  robot::model::RobotDescription robotDescription(urdfFile);
  robot::model::RobotState initState(robotDescription, 2); // from RobotState(const RobotDescription& robotDescription, size_t contactSize = 2);
  // so 2 is the contact size
  initState.setConfigurationToZero();

  const vector_t& initMpcState = interface.getInitialState();
  const auto& mpcModel = interface.getMpcRobotModel();
  initState.setRootPositionInWorldFrame(mpcModel.getBasePosition(initMpcState));

  vector_t mpcJointAngles = mpcModel.getJointAngles(initMpcState);
  // Todo set non zero orientation;
  std::vector<robot::joint_index_t> mpcJointIndices = robotDescription.getJointIndices(interface.modelSettings().mpcModelJointNames);
  for (size_t i = 0; i < mpcJointIndices.size(); i++) {
    initState.setJointPosition(mpcJointIndices[i], mpcJointAngles[i]);
  }

  std::cerr << "initState: " << initState.getRootPositionInWorldFrame().transpose() << std::endl;

  
  robot::mujoco_ros2_control_interface::MujocoSimConfig config;

  config.scenePath = mjxFile;
  config.verbose = true;
  config.initStatePtr_ = std::make_shared<robot::model::RobotState>(std::move(initState));

  // robot interface
  // // Note:
  // //class MujocoRos2ControlInterface : public robot::model::RobotHWInterfaceBase
  // // class MujocoRos2ControlInterface inherits from RobotHWInterfaceBase, 
  // // which means it implements the interface for a robot hardware. 
  // // This includes methods for getting the robot's current state, applying joint actions, and so on. 
  // // By creating an instance of MujocoRos2ControlInterface, 
  // we are essentially creating a simulated robot that we can interact with 
  // using the same interface we would use for real hardware. 
  // This allows us to test our MPC controller in simulation before deploying it on the actual robot.
  
  robot::mujoco_ros2_control_interface::MujocoRos2ControlInterface robotInterface(config, urdfFile);

  // MRT controller
  CentroidalMpcMrtJointController mpcJointController(robotInterface.getRobotDescription(), interface.modelSettings(),
                                                     interface.getMpcRobotModel(), mpc, interface.getPinocchioInterface(),
                                                     interface.mpcSettings().mpcDesiredFrequency_, humanoidVisualizer);
  
  // size_t mrtDeltaTMicroSeconds_ = 1000000 / (interface.mpcSettings().mrtDesiredFrequency_);
  size_t mrtDeltaTMicroSeconds_ = 1000000 / (500);
  // robotInterface.updateRobotStateFromSubscribedTopics(JointState_msg, Imu_msg); // Update the robot state from the subscribed joint states and imu data

  mpcJointController.startMpcThread(robotInterface.getRobotState());

  while (!mpcJointController.ready()) {
    RCLCPP_INFO_STREAM(nodeHandle->get_logger(), "Waiting for initial MPC policy...");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // print message for debugging
    RCLCPP_INFO_STREAM(nodeHandle->get_logger(), "Waiting for initial MPC policy...");
  }
  RCLCPP_INFO_STREAM(nodeHandle->get_logger(), "Initial MPC policy received.");
  std::cout << "Initial MPC policy received. " << std::endl;


  // Set up 200Hz publishing rate
  const double publish_rate_hz = 200.0;
  const auto publish_period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / publish_rate_hz));
  auto next_publish_time = std::chrono::steady_clock::now();
  rclcpp::spin_some(nodeHandle);

  while (true)
  {
    robotInterface.updateInterfaceStateFromRobot();
    // mpcJointController.computeJointControlAction(0.0, robotInterface.getRobotState(), robotInterface.getRobotJointAction());
    // robotInterface.applyJointAction();

    // Construct and publish JointState message from robotInterface.getRobotState()
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = nodeHandle->now();

    const auto& robotState = robotInterface.getRobotState();
    const auto& robotDescription = robotInterface.getRobotDescription();
    const auto& jointNames = robotDescription.getJointNames();

    joint_state_msg.name = jointNames;
    joint_state_msg.position.resize(jointNames.size());
    joint_state_msg.velocity.resize(jointNames.size());
    joint_state_msg.effort.resize(jointNames.size());

    for (size_t i = 0; i < jointNames.size(); ++i) {
      size_t jointId = robotDescription.getJointIndex(jointNames[i]);
      joint_state_msg.position[i] = robotState.getJointPosition(jointId);
      joint_state_msg.velocity[i] = robotState.getJointVelocity(jointId);
      // If you have effort data, set it here. Otherwise, leave as default (0.0)
      // joint_state_msg.effort[i] = robotState.getJointEffort(jointId);
    }

    publisher_joint_states->publish(joint_state_msg);

    rclcpp::spin_some(nodeHandle);

    next_publish_time += publish_period;
    std::this_thread::sleep_until(next_publish_time);
  }

  return 0;
}
