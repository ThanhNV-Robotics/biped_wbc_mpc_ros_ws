#include <robot_state_visualization/standing_balance_control.hpp> //included from control_actions package
#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_state_visualization/mujoco_rendering.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>


// MuJoCo data structures
mjModel *mj_model = nullptr;
mjData *mj_data = nullptr;
std::unordered_map<std::string, int> joint_qpos_index; // (joint_name, qpos_index)

// qpos start address of the torso free joint (-1 if not found)
int torso_free_qpos_adr = -1;

// joint names in centroidal state order (indices 12..23)
const std::vector<std::string> kCentroidalJointNames = {
  "left_hip_pitch_joint",  "left_hip_roll_joint",  "left_hip_yaw_joint",
  "left_knee_pitch_joint", "left_ankle_roll_joint", "left_ankle_pitch_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint",  "right_hip_yaw_joint",
  "right_knee_pitch_joint","right_ankle_roll_joint","right_ankle_pitch_joint"
};
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> filtered_acceleration_publisher;
// Real-time control parameters
const double SIMULATION_RATE = 1000.0;  // Hz 
const double RENDERING_RATE = 60.0;    // Hz
const double CAMERA_RATE = 6.0;        // Hz

const double SIMULATION_DT = 1.0 / SIMULATION_RATE;
const double RENDERING_DT = 1.0 / RENDERING_RATE;

// Vector3d represent CoM position
Eigen::Vector3d com;
//Vector2d represent ZMP position
Eigen::Vector3d zmp_position;
//vector3d represent acceleration
Eigen::Vector3d acceleration;
Eigen::Vector3d filtered_acceleration;

using namespace control_actions;
StandingBalanceController standing_balance_controller(250);

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  

  // set torso orientation in mujoco
  // Get torso free joint id
  int body_id = mj_name2id(mj_model, mjOBJ_BODY, "Link0_Torso");
  if (body_id < 0) return;

  int jnt_id = mj_model->body_jntadr[body_id];
  if (mj_model->jnt_type[jnt_id] != mjJNT_FREE) {
    RCLCPP_WARN(rclcpp::get_logger("imu_callback"),
                "Link0_Torso is not a free joint, cannot set quaternion directly!");
    return;
  }

  int qpos_adr = mj_model->jnt_qposadr[jnt_id];

  // Position: keep current
  // Orientation: reorder ROS (x,y,z,w) → MuJoCo (w,x,y,z)
  mj_data->qpos[qpos_adr + 3] = msg->orientation.w;
  mj_data->qpos[qpos_adr + 4] = msg->orientation.x;
  mj_data->qpos[qpos_adr + 5] = msg->orientation.y;
  mj_data->qpos[qpos_adr + 6] = msg->orientation.z;

  //store acceleration measurement to acceleration
  acceleration[0] = msg->linear_acceleration.x;
  acceleration[1] = msg->linear_acceleration.y;
  acceleration[2] = msg->linear_acceleration.z;

  // filt acceleration data and publish it
  filtered_acceleration = standing_balance_controller.kalman_filt(acceleration);
  sensor_msgs::msg::Imu filtered_acc_msg;
  filtered_acc_msg.linear_acceleration.x = filtered_acceleration[0];
  filtered_acc_msg.linear_acceleration.y = filtered_acceleration[1];
  filtered_acc_msg.linear_acceleration.z = filtered_acceleration[2];
  if (filtered_acceleration_publisher) {
    filtered_acceleration_publisher->publish(filtered_acc_msg);
  }
}

void visualize_com()
{
  // visulize CoM position  
  // mju_copy3 (com, mj_data->subtree_com); //index = 0 -> com of the whole model
  for (int i = 0; i < 3; ++i) {
    com[i] = mj_data->subtree_com[i]; // 
}
  int site_id = mj_name2id(mj_model, mjOBJ_SITE, "com");    

  if (site_id >= 0)
  {
    // Set site position = CoM
    mju_copy3(mj_data->site_xpos + 3 * site_id, com.data());
  }
}

void visualize_zmp ()
{
  // // zmp_position[0] = com[0] - (com[2]/9.81) * acceleration[0];// acceleration measured in m/s2
  // // zmp_position[1] = com[1] - (com[2]/9.81) * acceleration[1];// acceleration measured in m/s2

  // zmp_position[0] = com[0] - (com[2]) * acceleration[0];// acceleration measured in g
  // zmp_position[1] = com[1] - (com[2]) * acceleration[1];// acceleration measured in g
  // zmp_position[2] = 0.0; // on the ground

  // To debug standing controller's zmp calculation


  
  zmp_position = standing_balance_controller.compute_zmp(com, filtered_acceleration);

  // set posiiton to <site name="ZMP_est" pos="0 0 0" size="0.02" rgba="0 1 0 1"/>
  int site_id = mj_name2id(mj_model, mjOBJ_SITE, "ZMP_est");
  if (site_id >= 0)
  {
    // Set site position = zmp position
    mju_copy3(mj_data->site_xpos + 3 * site_id, zmp_position.data());
  }
}
// MPC observation callback: unpack centroidal state → base pose + joints → MuJoCo qpos
//
// Centroidal state layout:
//   [0..5]   normalized centroidal momentum (unused)
//   [6..8]   base position (x, y, z)
//   [9..11]  Euler ZYX (yaw, pitch, roll)
//   [12..23] joint angles (order = kCentroidalJointNames)
void mpc_observation_callback(const ocs2_msgs::msg::MpcObservation::SharedPtr msg)
{
  const auto& v = msg->state.value;
  if (static_cast<int>(v.size()) < 12 + static_cast<int>(kCentroidalJointNames.size())) return;

  const double yaw = v[9], pitch = v[10], roll = v[11];

  // Euler ZYX → quaternion
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());

  // Base pose via free joint (MuJoCo quaternion order: w, x, y, z)
  if (torso_free_qpos_adr >= 0) {
    mj_data->qpos[torso_free_qpos_adr + 0] = v[6];   // x
    mj_data->qpos[torso_free_qpos_adr + 1] = v[7];   // y
    mj_data->qpos[torso_free_qpos_adr + 2] = v[8];   // z
    mj_data->qpos[torso_free_qpos_adr + 3] = q.w();  // MuJoCo quaternion order: w, x, y, z
    mj_data->qpos[torso_free_qpos_adr + 4] = q.x();
    mj_data->qpos[torso_free_qpos_adr + 5] = q.y();
    mj_data->qpos[torso_free_qpos_adr + 6] = q.z();
  }

  // Joint angles
  for (size_t i = 0; i < kCentroidalJointNames.size(); ++i) {
    auto it = joint_qpos_index.find(kCentroidalJointNames[i]);
    if (it != joint_qpos_index.end()) {
      mj_data->qpos[it->second] = static_cast<double>(v[12 + i]);
    }
  }
}

// JointState callback: copy positions into MuJoCo
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); i++) {
    auto it = joint_qpos_index.find(msg->name[i]);
    if (it != joint_qpos_index.end()) {
      int qpos_id = it->second;
      mj_data->qpos[qpos_id] = msg->position[i];
      if (i < msg->velocity.size()) {
        mj_data->qvel[qpos_id] = msg->velocity[i];
      }
    }
  }
  // visulize CoM position
  mjtNum com[3];
  mju_copy3 (com, mj_data->subtree_com); //index = 0 -> com of the whole model
  int site_id = mj_name2id(mj_model, mjOBJ_SITE, "com");    

  if (site_id >= 0)
  {
    // Set site position = CoM
    mju_copy3(mj_data->site_xpos + 3 * site_id, com);
  }
}

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "robot_state_visualization_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  filtered_acceleration_publisher = node->create_publisher<sensor_msgs::msg::Imu>(
    "/filtered_acceleration", 10);

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (
    std::strlen(model_path.c_str()) > 4 &&
    !std::strcmp(model_path.c_str() + std::strlen(model_path.c_str()) - 4, ".mjb"))
  {
    mj_model = mj_loadModel(model_path.c_str(), 0);
  }
  else
  {
    mj_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
    
  }
  if (!mj_model)
  {
    mju_error("Load model error: %s", error);
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  }
  
  // make data
  mj_data = mj_makeData(mj_model);

  // Build mapping: joint name → qpos index
  for (int j = 0; j < mj_model->njnt; j++) {
    std::string jname = mj_model->names + mj_model->name_jntadr[j];
    int qpos_adr = mj_model->jnt_qposadr[j];
    joint_qpos_index[jname] = qpos_adr;
  }

  // Find torso free joint for base pose (try both capitalisation variants)
  for (const char* torso_name : {"link0_torso", "Link0_Torso"}) {
    int body_id = mj_name2id(mj_model, mjOBJ_BODY, torso_name);
    if (body_id < 0) continue;
    int jnt_id = mj_model->body_jntadr[body_id];
    if (jnt_id >= 0 && mj_model->jnt_type[jnt_id] == mjJNT_FREE) {
      torso_free_qpos_adr = mj_model->jnt_qposadr[jnt_id];
      RCLCPP_INFO_STREAM(node->get_logger(),
          "Torso free joint found (body='" << torso_name << "'), qpos_adr=" << torso_free_qpos_adr);
      break;
    }
  }
  if (torso_free_qpos_adr < 0) {
    RCLCPP_WARN_STREAM(node->get_logger(), "No free joint found on torso body — base pose won't be visualised.");
  }

  // initialize mujoco control
  //auto mujoco_control = mujoco_ros2_control::MujocoRos2Control(node, mj_model, mj_data);

  //mujoco_control.init();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // initialize mujoco visualization environment for rendering and cameras
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }
  auto rendering = robot_state_visualization::MujocoRendering::get_instance();
  rendering->init(mj_model, mj_data);
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");

  // run main loop, target real-time simulation and 60 fps rendering with cameras around 6 hz
    // Real-time timing variables
  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_simulation_time = start_time;
  auto last_rendering_time = start_time;
  auto last_camera_time = start_time;

  // // Create a subscription to subsribe to /joint_states
  // auto subscription_joint_states = node->create_subscription<sensor_msgs::msg::JointState>(
  //     "/joint_states", 10, joint_state_callback );

  // // create a subscription to subscribe to /imu
  // auto subscription_imu = node->create_subscription<sensor_msgs::msg::Imu>(
  //     "/imu_sensor_broadcaster/imu", 10, imu_callback );

  // Subscribe to MPC observation: estimated base pose + joint angles from state estimator
  auto subscription_mpc_obs = node->create_subscription<ocs2_msgs::msg::MpcObservation>(
      "/legged_robot_mpc_observation", rclcpp::SensorDataQoS(), mpc_observation_callback);
  
  while (rclcpp::ok() && !rendering->is_close_flag_raised())
  //while (!rendering->is_close_flag_raised())
  {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed_simulation = std::chrono::duration<double>(current_time - last_simulation_time).count();
    auto elapsed_rendering = std::chrono::duration<double>(current_time - last_rendering_time).count();
    auto elapsed_camera = std::chrono::duration<double>(current_time - last_camera_time).count();
    // Update simulation at fixed rate
    if (elapsed_simulation >= SIMULATION_DT)
    {
      mj_forward(mj_model, mj_data);
      last_simulation_time = current_time;
    }

    // Update rendering at fixed rate
    if (elapsed_rendering >= RENDERING_DT)
    {
      visualize_com();
      visualize_zmp();
      rendering->update();
      last_rendering_time = current_time;
    }
    rclcpp::spin_some(node); //spin(node) not work
    // mjtNum simstart = mj_data->time;
    // while (mj_data->time - simstart < 1.0/60.0) {
    //   mj_step(mj_model, mj_data);      
    // }
    // rclcpp::spin_some(node); //spin(node) not work
    // rendering->update();
  }

  rendering->close();
  //cameras->close();

  // free MuJoCo model and data
  mj_deleteData(mj_data);
  mj_deleteModel(mj_model);

  return 1;
}
