// This is to subscribe to the joint state and imu topic published by the mujoco_ros2_control_interface and use that as the state feedback for the MPC.
#pragma once

#include <iostream>
#include <string>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <atomic>
#include <chrono>
#include <ctime>
#include <mutex>
#include <thread>

#include <Eigen/Dense>

#include <robot_model/RobotState.h>
#include "mujoco_sim_interface/MujocoUtils.h"
#include "robot_core/Types.h"
#include "robot_model/RobotHWInterfaceBase.h"

// include rclpp for subscribing to joint states and imu published by mujoco_ros2_control_interface
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace robot::mujoco_ros2_control_interface {

struct MujocoSimConfig {
  std::string scenePath;
  std::shared_ptr<model::RobotState> initStatePtr_;
  double dt{0.0005};
  double renderFrequencyHz{60.0};
  bool headless{false};
  bool verbose{false};
};

class MujocoRos2ControlInterface : public robot::model::RobotHWInterfaceBase {
 public:
  MujocoRos2ControlInterface(const MujocoSimConfig& config, const std::string& urdfPath);

  /** Destructor */
  ~MujocoRos2ControlInterface();

  // Todo Manu also reset environment
  void reset();

  const mjModel* getModel() const { return mujocoModel_; }

  const MujocoSimConfig& getConfig() const { return config_; }

  void updateRobotStateFromSubscribedTopics(const sensor_msgs::msg::JointState::SharedPtr jointStateMsg,
                                      const sensor_msgs::msg::Imu::SharedPtr imuMsg);

    // return an order map of <joint_name, joint_position> from robotStateInternal_
  std::unordered_map<std::string, double> getJointPositionMap() const;
  

 private: 

  // imu subscriber attributes
  // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
  // joint state subscriber attributes
  // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription;
  // // Subsciber to joint states published by mujoco_ros2_control_interface
  // // Subscription to joint states
  // auto subscription_joint_states = node->create_subscription<sensor_msgs::msg::JointState>(
  //     "/joint_states", 10, joint_state_callback );
  // // Subscription to imu
  // auto subscription_imu = node->create_subscription<sensor_msgs::msg::Imu>(
  //     "/imu_sensor_broadcaster/imu", 10, imu_callback );

  // void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg); // callback for imu subscriber

  // void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg); // callback for joint state subscriber

  // Function to update Robot State from joint states and imu published by mujoco_ros2_control
  void updateTopicRobotState();

  

  void setupJointIndexMaps();

  void setSimState(const model::RobotState& robotState);

  void printModelInfo();

  MujocoSimConfig config_;

  model::RobotState robotStateInternal_;
  mjtNum* qpos_init_;  // position                                         (nq x 1)
  mjtNum* qvel_init_;
  model::RobotJointAction robotJointActionInternal_;

  size_t timeStepMicro_;
  double simStart_;
  size_t nActiveJoints_;
  size_t nActuators_;
  std::vector<std::string> activeMuJoCoJointNames_;
  std::vector<std::string> activeMuJoCoActuatorNames_;
  std::vector<joint_index_t> activeRobotJointStateIndices_;
  std::vector<joint_index_t> activeRobotActuatorIndices_;

  mjModel* mujocoModel_ = NULL;
  mjData* mujocoData_ = NULL;
  mjContact* mujocoContact_ = NULL;
  // mjfSensor mujocoSenor_;

  bool simInit_;
  const bool headless_;
  const bool verbose_;
  std::atomic<bool> terminate_{false};
  std::atomic<bool> guiInitialized_{false};

  mutable std::mutex mujocoMutex_;  // Used to access mujoco model and data accross simulation and render threads.


  std::chrono::high_resolution_clock::time_point lastRealTime_;

  std::shared_ptr<rclcpp::Node> RobotInterfaceHandleNode = rclcpp::Node::make_shared(
    "robot_state_visualization_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
};

}  // namespace robot::mujoco_ros2_control_interface
