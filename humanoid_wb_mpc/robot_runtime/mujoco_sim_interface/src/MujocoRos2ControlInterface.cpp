#include "mujoco_sim_interface/MujocoRos2ControlInterface.h"

#include <cerrno>
#include <cstring>
#include <limits>
#include <stdexcept>

#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

namespace robot::mujoco_ros2_control_interface {

// MjState::MjState(const mjModel* mujocoModel_) : data(mj_makeData(mujocoModel_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

MujocoRos2ControlInterface::MujocoRos2ControlInterface(const MujocoSimConfig& config, const std::string& urdfPath)
    : RobotHWInterfaceBase(urdfPath),
      config_(config),
      robotStateInternal_(model::RobotState(this->getRobotDescription())),
      robotJointActionInternal_(model::RobotJointAction(this->getRobotDescription())),
      headless_(config.headless),
      verbose_(config.verbose) {

  RobotInterfaceHandleNode = rclcpp::Node::make_shared(
    "robot_state_visualization_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).use_global_arguments(false)); // init a node to handle to topic subscription


  lastRealTime_ = std::chrono::high_resolution_clock::now();
  const int errstr_sz = 1000;  // Define the size of the error buffer
  char errstr[errstr_sz];      // Declare the error string buffer

  // option 1: parse and compile XML from file
  std::cerr << "\n=== Loading MuJoCo XML ===" << std::endl;
  std::cerr << "File path: " << config.scenePath << std::endl;
  
  mujocoModel_ = mj_loadXML(config.scenePath.c_str(), NULL, errstr, errstr_sz);
  if (!mujocoModel_) {
    std::cerr << "Could not load MuJoCo model: " << config.scenePath << ". Error: " << std::strerror(errno) << std::endl;
    std::cerr << "MuJoCo error string: " << errstr << std::endl;
    throw std::runtime_error("Could not load MuJoCo: " + std::string(std::strerror(errno)));
  }
  
  if (strlen(errstr) > 0) {
    std::cerr << "MuJoCo warnings/errors during XML load: " << errstr << std::endl;
  }
  std::cerr << "MuJoCo model loaded successfully" << std::endl;
  std::cerr << "Number of joints in model (njnt): " << mujocoModel_->njnt << std::endl;

  // Create data
  mujocoData_ = mj_makeData(mujocoModel_);

  /* initialize random seed: */
  srand(time(NULL));

  mujocoContact_ = mujocoData_->contact;

  simStart_ = mujocoData_->time;

  // assert(nActiveJoints_ == neo_definitions::FULL_NEO_JOINT_DIM);

  mujocoModel_->opt.timestep = config_.dt;

  timeStepMicro_ = static_cast<size_t>(config_.dt * 1000000);

  if (verbose_) printModelInfo();

  setupJointIndexMaps();

  model::RobotState initRobotState(getRobotDescription(), 2);

  if (config_.initStatePtr_ != nullptr) {
    initRobotState = *config.initStatePtr_;
  } else {
    initRobotState.setConfigurationToZero();
    initRobotState.setRootPositionInWorldFrame(vector3_t(0.0, 0.0, 1.0));
  }
  
  // // DEBUG: Print what joints RobotDescription knows about
  // std::cerr << "\n=== RobotDescription vs MuJoCo Joint Names ===" << std::endl;
  // std::cerr << "activeRobotJointStateIndices_ (from RobotDescription.getJointIndices()):" << std::endl;
  // for (size_t i = 0; i < activeMuJoCoJointNames_.size(); ++i) {
  //   std::cerr << "  MuJoCo[" << i << "] " << activeMuJoCoJointNames_[i] 
  //             << " -> RobotState[" << activeRobotJointStateIndices_[i] << "]" << std::endl;
  // }
  
  setSimState(initRobotState);

  // Add default joint damping
  scalar_t defaultJointDamping = 10.0;

  for (int i = 6; i < mujocoModel_->nv; ++i) {
    std::string mjJointName(&mujocoModel_->names[mujocoModel_->name_jntadr[mujocoModel_->dof_jntid[i]]]);
    std::cerr << "mjJointName: " << mjJointName << std::endl;
    mujocoModel_->dof_damping[i] = defaultJointDamping;
  }

  qpos_init_ = new mjtNum[mujocoModel_->nq];
  qvel_init_ = new mjtNum[mujocoModel_->nv];

  // Safe init state for resets
  memcpy(qpos_init_, mujocoData_->qpos, mujocoModel_->nq * sizeof(mjtNum));
  memcpy(qvel_init_, mujocoData_->qvel, mujocoModel_->nv * sizeof(mjtNum));

  // create a subscription to robot states published by mujoco_ros2_control_interface
  // Subscription to joint states
  // joint_state_subscription = RobotInterfaceHandleNode->create_subscription<sensor_msgs::msg::JointState>(
      //  "/joint_states", 10, std::bind(&MujocoRos2ControlInterface::joint_state_callback, this, std::placeholders::_1));

    // Subscription to imu
  // imu_subscription = RobotInterfaceHandleNode->create_subscription<sensor_msgs::msg::Imu>(
      // "/imu_sensor_broadcaster/imu", 10, std::bind(&MujocoRos2ControlInterface::imu_callback, this, std::placeholders::_1) );


  // // Make sure the init state is propagated throughout the RobotInterface.
  // updateThreadSafeRobotState();
  // updateInterfaceStateFromRobot();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// Update robot state from subscribed topics (joint states and imu)
void MujocoRos2ControlInterface::updateRobotStateFromSubscribedTopics(const sensor_msgs::msg::JointState::SharedPtr jointStateMsg,
                                                              const sensor_msgs::msg::Imu::SharedPtr imuMsg) {

// // update joint states
//   static bool printed_joint_info = false;
//   if (!printed_joint_info) {
//     std::cerr << "\n=== Joint State Message Info ===" << std::endl;
//     std::cerr << "Received " << jointStateMsg->name.size() << " joints from topic:" << std::endl;
//     for (size_t i = 0; i < jointStateMsg->name.size(); ++i) {
//       std::cerr << "  [" << i << "] " << jointStateMsg->name[i] << std::endl;
//     }
//     std::cerr << "\nRobot Description joints:" << std::endl;
//     const auto& allJointNames = getRobotDescription().getJointNames();
//     for (size_t i = 0; i < allJointNames.size(); ++i) {
//       std::cerr << "  [" << i << "] " << allJointNames[i] << std::endl;
//     }
//     printed_joint_info = true;
//   }

  for (size_t i = 0; i < jointStateMsg->name.size(); ++i) {
    const std::string& jointName = jointStateMsg->name[i];
    double jointPosition = jointStateMsg->position[i];
    double jointVelocity = jointStateMsg->velocity[i];

    // Update the robot state with the received joint states
    if (getRobotDescription().containsJoint(jointName)) {
      size_t jointIndex = getRobotDescription().getJointIndex(jointName);
      robotStateInternal_.setJointPosition(jointIndex, jointPosition);
      robotStateInternal_.setJointVelocity(jointIndex, jointVelocity);
    }
  }

  // update imu data
  double angularVelX = imuMsg->angular_velocity.x;
  double angularVelY = imuMsg->angular_velocity.y;
  double angularVelZ = imuMsg->angular_velocity.z;

  // Update the robot state with the received IMU data
  robotStateInternal_.setRootAngularVelocityInLocalFrame(vector3_t(angularVelX, angularVelY, angularVelZ));

  // imu quaternion data is in the form of (x, y, z, w), but we need to convert it to (w, x, y, z) for mujoco
  quaternion_t quat_l_to_w(imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z);
  robotStateInternal_.setRootRotationLocalToWorldFrame(quat_l_to_w);

    // Fix later
  // bool leftFootContact = (mujocoData_->sensordata[left_foot_touch_sensor_addr_] > 0.1);
  // bool rightFootContact = (mujocoData_->sensordata[right_foot_touch_sensor_addr_] > 0.1);
  bool leftFootContact = true;
  bool rightFootContact = true;

  robotStateInternal_.setContactFlag(0, leftFootContact);
  robotStateInternal_.setContactFlag(1, rightFootContact);

  // Estimate base height using Pinocchio FK if the estimator has been initialised.
  // Build q with base at origin + IMU orientation + current joint angles, run FK,
  // then base_z = -(lowest contact frame z), so the lowest foot lands at z = 0.
  if (heightEstPinocchioInterface_.has_value()) {
    const int nJoints = heightEstPinocchioInterface_->getModel().nq - 6;  // subtract floating-base dof (Translation + EulerZYX = 6)
    ocs2::vector_t q(6 + nJoints);
    q[0] = 0.0;  // base x
    q[1] = 0.0;  // base y
    q[2] = 0.0;  // base z (doesn't affect relative FK)
    // Pinocchio SphericalZYX uses Euler ZYX angles (z, y, x)
    const quaternion_t& quat = robotStateInternal_.getRootRotationLocalToWorldFrame();
    const ocs2::vector3_t eulerZYX = ocs2::humanoid::quaternionToEulerZYX(quat);
    q[3] = eulerZYX[0];  // theta_z
    q[4] = eulerZYX[1];  // theta_y
    q[5] = eulerZYX[2];  // theta_x

    // get joint angles from robot state in Pinocchio's expected joint order
    const auto& pinocchioModel = heightEstPinocchioInterface_->getModel();
    for (size_t i = 0; i < static_cast<size_t>(nJoints); ++i) {
      // Get joint name from Pinocchio model (skip universe at index 0, root_joint at index 1)
      const std::string& pinocchioJointName = pinocchioModel.names[i + 2];  // +2 to skip universe and root_joint
      
      // Find this joint in RobotDescription and get its angle
      if (getRobotDescription().containsJoint(pinocchioJointName)) {
        size_t robotStateIdx = getRobotDescription().getJointIndex(pinocchioJointName);
        q[6 + i] = robotStateInternal_.getJointPosition(robotStateIdx);
      } else {
        q[6 + i] = 0.0;  // Default to zero if joint not found
      }
    }

    // // DEBUG: Print joint configuration and orientation
    // static int debug_counter = 0;
    // if (debug_counter++ % 100 == 0) {  // Print every 100 iterations
    //   std::cerr << "\n=== FK Debug Info ===" << std::endl;
    //   std::cerr << "Orientation (quat x,y,z,w): " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << std::endl;
    //   std::cerr << "nActiveJoints: " << nActiveJoints_ << ", nJoints (Pinocchio): " << nJoints << std::endl;
      
    //   // Print Pinocchio joint names to identify which joint is missing
    //   if (debug_counter == 1) {
    //     std::cerr << "Pinocchio model joints (excluding floating base):" << std::endl;
    //     const auto& model = heightEstPinocchioInterface_->getModel();
    //     for (size_t jid = 1; jid < model.names.size(); ++jid) {  // Start from 1 to skip universe
    //       std::cerr << "  [" << jid-1 << "] " << model.names[jid] << std::endl;
    //     }
    //   }
      
    //   std::cerr << "Joint names and angles:" << std::endl;
    //   for (size_t i = 0; i < std::min(static_cast<size_t>(nJoints), nActiveJoints_); ++i) {
    //     std::string jointName = getRobotDescription().getJointName(activeRobotJointStateIndices_[i]);
    //     std::cerr << "  [" << i << "] " << jointName << ": " << q[6 + i] << std::endl;
    //   }
    //   std::cerr << "Contact frame names: ";
    //   for (const auto& name : heightEstContactFrameNames_) {
    //     std::cerr << name << " ";
    //   }
    //   std::cerr << std::endl;
    // }

    const auto footPositions = ocs2::humanoid::computeFramePositions(
        q, *heightEstPinocchioInterface_, heightEstContactFrameNames_);

    // // DEBUG: Print foot positions to see which one changes when leg moves
    // std::cerr << "Foot[0] z: " << footPositions[0][2] 
    //           << " | Foot[1] z: " << footPositions[1][2] << std::endl;

    double min_foot_z = std::numeric_limits<double>::max();
    // for (const auto& pos : footPositions) {
    //   min_foot_z = std::min(min_foot_z, pos[2]);
    // }
    
    min_foot_z = std::min(footPositions[0][2], footPositions[1][2]);
    // min_foot_z = footPositions[1][2]; // TODO: fix this when we have proper contact info, for now just use left foot height as base height estimate
    const double base_z = -min_foot_z;
    
    // // DEBUG: Directly use left foot z position as base height (footPositions[1] is left foot)
    // // const double base_z = footPositions[1][2]; // to check left leg or right leg
    // const double base_z = footPositions[0][2]; // to check left leg or right leg
    
    // std::cerr << "Setting base_z to: " << base_z << std::endl;
    robotStateInternal_.setRootPositionInWorldFrame(
        vector3_t(robotStateInternal_.getRootPositionInWorldFrame()[0],
                  robotStateInternal_.getRootPositionInWorldFrame()[1],
                  base_z));
  }

  // Push into the thread-safe store so that updateInterfaceStateFromRobot() + getRobotState() see the latest topic data.
  threadSafeRobotState_.set(robotStateInternal_);
}

void MujocoRos2ControlInterface::initBaseHeightEstimator(
    ocs2::PinocchioInterface pinocchioInterface,
    std::vector<std::string> contactFrameNames) {
  heightEstPinocchioInterface_ = std::move(pinocchioInterface);
  heightEstContactFrameNames_  = std::move(contactFrameNames);
}

// Impelement of std::unordered_map<std::string, double> getJointPositionMap() const;

std::unordered_map<std::string, double> MujocoRos2ControlInterface::getJointPositionMap() const {
  std::unordered_map<std::string, double> jointPositionMap;

  for (size_t i = 0; i < nActiveJoints_; ++i) {
    std::string jointName = getRobotDescription().getJointName(activeRobotJointStateIndices_[i]);
    double jointPosition = robotStateInternal_.getJointPosition(activeRobotJointStateIndices_[i]);
    jointPositionMap[jointName] = jointPosition;
  }

  return jointPositionMap;
}
  

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// // callback for imu subsciber
// void MujocoRos2ControlInterface::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//   // print a message for debugging
//   RCLCPP_INFO_STREAM(RobotInterfaceHandleNode->get_logger(), "Received IMU message: " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << " - Linear Acceleration: [" << msg->linear_acceleration.x << ", " << msg->linear_acceleration.y << ", " << msg->linear_acceleration.z << "] - Angular Velocity: [" << msg->angular_velocity.x << ", " << msg->angular_velocity.y << ", " << msg->angular_velocity.z << "]");
//   return;
// }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// callback for joint state subscriber
// void MujocoRos2ControlInterface::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
// {
//   return;
// }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

MujocoRos2ControlInterface::~MujocoRos2ControlInterface() {
  terminate_.store(true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoRos2ControlInterface::reset() {
  memcpy(mujocoData_->qpos, qpos_init_, mujocoModel_->nq * sizeof(mjtNum));
  memcpy(mujocoData_->qvel, qvel_init_, mujocoModel_->nv * sizeof(mjtNum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoRos2ControlInterface::setupJointIndexMaps() {
  // Mujoco to Robot joints
  std::cerr << "\n=== MuJoCo Model Joints ===" << std::endl;
  for (int i = 1; i < mujocoModel_->njnt; ++i) {
    // Get the joint name
    const std::string jointName(&mujocoModel_->names[mujocoModel_->name_jntadr[i]]);
    std::cerr << "  [" << i << "] " << jointName;
    if (getRobotDescription().containsJoint(jointName)) {
      activeMuJoCoJointNames_.emplace_back(jointName);
      activeMuJoCoJointIndices_.emplace_back(i);  // Store MuJoCo joint index
      std::cerr << " ✓ (matched)" << std::endl;
    } else {
      std::cerr << " ✗ (NOT in RobotDescription)" << std::endl;
      std::cerr << "WARNING: Joint contained in mujoco xml not exposed to RobotHWInterface: " << jointName << std::endl;
    }
  }
  std::cerr << "Total MuJoCo joints: " << (mujocoModel_->njnt - 1) << std::endl;
  std::cerr << "Matched joints: " << activeMuJoCoJointNames_.size() << std::endl;

  activeRobotJointStateIndices_ = getRobotDescription().getJointIndices(activeMuJoCoJointNames_);

  // Mujoco to robot actuators
  for (int i = 0; i < mujocoModel_->nu; ++i) {
    const std::string actuator_name = mj_id2name(mujocoModel_, mjOBJ_ACTUATOR, i);

    if (getRobotDescription().containsJoint(actuator_name)) {
      activeMuJoCoActuatorNames_.emplace_back(actuator_name);
    } else {
      std::cerr << "WARNING: Actuator contained in mujoco xml not be commanded through RobotHWInterface: " << actuator_name << std::endl;
    }
  }

  activeRobotActuatorIndices_ = getRobotDescription().getJointIndices(activeMuJoCoActuatorNames_);

  nActiveJoints_ = activeRobotJointStateIndices_.size();
  nActuators_ = activeRobotActuatorIndices_.size();
  if (verbose_) {
    std::cerr << "Initialized " << nActiveJoints_ << " active Joints" << std::endl;
    std::cerr << "Initialized " << nActuators_ << " active Actuators" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoRos2ControlInterface::printModelInfo() {
  std::cerr << "timeStepMicro_: " << timeStepMicro_ << std::endl;

  std::cerr << "njnt: " << mujocoModel_->njnt << std::endl;
  std::cerr << "nq: " << mujocoModel_->nq << std::endl;
  std::cerr << "nv: " << mujocoModel_->nv << std::endl;
  std::cerr << "nu: " << mujocoModel_->nu << std::endl;

  for (int i = 0; i < mujocoModel_->nbody; ++i) {
    std::string bodyName(&mujocoModel_->names[mujocoModel_->name_bodyadr[i]]);
    std::cerr << "Body " << i << ": " << bodyName << std::endl;

    std::cerr << "  Position: ";
    for (size_t j = 0; j < 3; ++j) {
      std::cerr << mujocoData_->xpos[i * 3 + j] << " ";
    }
    std::cerr << std::endl;

    // Print orientation quaternion
    std::cerr << "  Orientation (Quaternion): ";
    for (size_t j = 0; j < 4; ++j) {
      std::cerr << mujocoData_->xquat[i * 4 + j] << " ";
    }
    std::cerr << std::endl;
  }

  std::string jointName(&mujocoModel_->names[mujocoModel_->name_jntadr[0]]);

  // Print the information
  std::cerr << "Joint Name: " << jointName << std::endl;
  std::cerr << "Position: " << mujocoData_->qpos[0] << " " << mujocoData_->qpos[1] << " " << mujocoData_->qpos[2] << " "
            << mujocoData_->qpos[3] << " " << mujocoData_->qpos[4] << " " << mujocoData_->qpos[5] << " " << mujocoData_->qpos[6]
            << std::endl;
  std::cerr << "Velocity: " << mujocoData_->qvel[0] << " " << mujocoData_->qvel[1] << " " << mujocoData_->qvel[2] << " "
            << mujocoData_->qvel[3] << " " << mujocoData_->qvel[4] << " " << mujocoData_->qvel[5] << std::endl;

  // Print joint names, positions, and velocities
  for (int i = 1; i < mujocoModel_->njnt; ++i) {
    // Get the joint name
    std::string jointName(&mujocoModel_->names[mujocoModel_->name_jntadr[i]]);

    // Get the joint position and velocity using proper MuJoCo addressing
    int qpos_addr = mujocoModel_->jnt_qposadr[i];
    int dof_addr = mujocoModel_->jnt_dofadr[i];
    double jointPos = mujocoData_->qpos[qpos_addr];
    double jointVel = mujocoData_->qvel[dof_addr];

    // Print the information
    std::cerr << "Joint Name: " << jointName << ", Position: " << jointPos << ", Velocity: " << jointVel << std::endl;
  }

  // Calculate total mass
  scalar_t totalMass = 0.0;
  for (int i = 0; i < mujocoModel_->nbody; i++) {
    totalMass += mujocoModel_->body_mass[i];
  }
  std::cerr << "Total MuJoCo model mass: " << totalMass << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void MujocoRos2ControlInterface::setSimState(const model::RobotState& robotState) {
  // Root Pose
  vector3_t rootPosition = robotState.getRootPositionInWorldFrame();
  quaternion_t quat_l_to_w = robotState.getRootRotationLocalToWorldFrame();

  mujocoData_->qpos[0] = rootPosition[0];
  mujocoData_->qpos[1] = rootPosition[1];
  mujocoData_->qpos[2] = rootPosition[2];
  mujocoData_->qpos[3] = quat_l_to_w.w();
  mujocoData_->qpos[4] = quat_l_to_w.x();
  mujocoData_->qpos[5] = quat_l_to_w.y();
  mujocoData_->qpos[6] = quat_l_to_w.z();

  // Root Velocity

  vector3_t root_vel_lin_world_frame = quat_l_to_w * robotState.getRootLinearVelocityInLocalFrame();
  vector3_t root_vel_ang_local_frame = robotState.getRootAngularVelocityInLocalFrame();

  mujocoData_->qvel[0] = root_vel_lin_world_frame[0];
  mujocoData_->qvel[1] = root_vel_lin_world_frame[1];
  mujocoData_->qvel[2] = root_vel_lin_world_frame[2];
  mujocoData_->qvel[3] = root_vel_ang_local_frame[0];
  mujocoData_->qvel[4] = root_vel_ang_local_frame[1];
  mujocoData_->qvel[5] = root_vel_ang_local_frame[2];

  // DEBUG: Print sizing info
  static bool printed_once = false;
  if (!printed_once) {
    std::cerr << "\n=== setSimState() Debug ===" << std::endl;
    std::cerr << "nActiveJoints_: " << nActiveJoints_ << std::endl;
    std::cerr << "activeMuJoCoJointIndices_.size(): " << activeMuJoCoJointIndices_.size() << std::endl;
    std::cerr << "activeRobotJointStateIndices_.size(): " << activeRobotJointStateIndices_.size() << std::endl;
    printed_once = true;
  }

  // Joint State - use proper MuJoCo addressing for each joint
  for (size_t i = 0; i < nActiveJoints_; ++i) {
    int mjJointIdx = activeMuJoCoJointIndices_[i];
    int qpos_addr = mujocoModel_->jnt_qposadr[mjJointIdx];
    int dof_addr = mujocoModel_->jnt_dofadr[mjJointIdx];
    
    // DEBUG: Print what we're about to access
    std::string jointName(&mujocoModel_->names[mujocoModel_->name_jntadr[mjJointIdx]]);
    std::cerr << "  [" << i << "] " << jointName 
              << " mjIdx=" << mjJointIdx 
              << " qpos_addr=" << qpos_addr 
              << " dof_addr=" << dof_addr
              << " robotStateIdx=" << activeRobotJointStateIndices_[i] << std::endl;
    
    mujocoData_->qpos[qpos_addr] = robotState.getJointPosition(activeRobotJointStateIndices_[i]);
    mujocoData_->qvel[dof_addr] = robotState.getJointVelocity(activeRobotJointStateIndices_[i]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

// Function to update Robot State from joint states and imu published by mujoco_ros2_control
void MujocoRos2ControlInterface::updateTopicRobotState() {
  // TODO 
  return;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

}  // namespace robot::mujoco_ros2_control_interface