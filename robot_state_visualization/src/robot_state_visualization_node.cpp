#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"

#include "robot_state_visualization/mujoco_cameras.hpp"
#include "robot_state_visualization/mujoco_rendering.hpp"
#include <unordered_map>

// MuJoCo data structure
mjModel *mj_model = nullptr;
mjData *mj_data = nullptr;
std::unordered_map<std::string, int> joint_qpos_index;
// main function
int main(int argc, const char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("robot_visulization_node",
                                                                   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
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

    RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
    // make data
    mj_data = mj_makeData(mj_model);

    // initialize mujoco visualization environment for rendering and cameras
    if (!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }
    auto rendering = robot_state_visualization::MujocoRendering::get_instance();
    rendering->init(mj_model, mj_data);
    RCLCPP_INFO_STREAM(node->get_logger(), "MuJoCo rendering has been successfully initialized");
    auto cameras = std::make_unique<robot_state_visualization::MujocoCameras>(node);
    cameras->init(mj_model);
    // run main loop, target real-time simulation and 60 fps rendering with cameras around 6 hz
    mjtNum last_cam_update = mj_data->time;
    while (rclcpp::ok() && !rendering->is_close_flag_raised())
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = mj_data->time;
        while (mj_data->time - simstart < 1.0 / 60.0)
        {
            mj_forward(mj_model, mj_data);
        }
        rendering->update();

        // Updating cameras at ~6 Hz
        // TODO(eholum): Break control and rendering into separate processes
        if (simstart - last_cam_update > 1.0 / 6.0)
        {
        cameras->update(mj_model, mj_data);
        last_cam_update = simstart;
        }
        rclcpp::spin(node);
    }
}