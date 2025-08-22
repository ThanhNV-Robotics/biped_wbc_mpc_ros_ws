#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_state_visualization/mujoco_rendering.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <string>
#include <vector>
// MuJoCo data structures
mjModel *mj_model = nullptr;
mjData *mj_data = nullptr;
std::unordered_map<std::string, int> joint_qpos_index; // (joint_name, qpos_index)

// Real-time control parameters
const double SIMULATION_RATE = 1000.0;  // Hz 
const double RENDERING_RATE = 60.0;    // Hz
const double CAMERA_RATE = 6.0;        // Hz

const double SIMULATION_DT = 1.0 / SIMULATION_RATE;
const double RENDERING_DT = 1.0 / RENDERING_RATE;

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
  // // Update derived states after modifying qpos/qvel
  // mj_forward(mj_model, mj_data);
}

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "robot_state_visualization_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  //std::shared_ptr<rclcpp::Node> node_ptr = & node;

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

  // Create a subscription
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, joint_state_callback );

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
