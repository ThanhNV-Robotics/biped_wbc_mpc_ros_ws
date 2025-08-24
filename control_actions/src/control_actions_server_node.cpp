#include <control_actions/control_actions_server.hpp>
#include <rclcpp/rclcpp.hpp>

mjModel *mj_model; // to read the robot configuration

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto control_action_server_node = std::make_shared<control_actions_cpp::ControlActionServer>();

    //check input parameters, shoulde include a path to mujoco model
    if (control_action_server_node->get_parameter("mujoco_model_path").as_string().empty())
    {
        RCLCPP_ERROR(control_action_server_node->get_logger(), "mujoco_model_path parameter is not set");
        return 1;
    }
    auto model_path = control_action_server_node->get_parameter("mujoco_model_path").as_string();
    // load and compile the mujoco model
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
    // check if the model is loaded successfully
    if (!mj_model)
    {
        RCLCPP_ERROR(control_action_server_node->get_logger(), "Control Action Server: Could not load mujoco model: %s", error);
        return 1;
    }
    RCLCPP_INFO_STREAM(control_action_server_node->get_logger(), " Control Action Server: successfully loaded !");
    // load mj_model to node class
    control_action_server_node->set_mujoco_model(mj_model);
    // set joint limits
    std::unordered_map<std::string, std::pair<double, double>> joint_limits;
    for (int j = 0; j < mj_model->njnt; j++) // loop over all joints
    {
        std::string jname = mj_model->names + mj_model->name_jntadr[j];
        double lower_limit = mj_model->jnt_range[2 * j];
        double upper_limit = mj_model->jnt_range[2 * j + 1];
        joint_limits[jname] = std::make_pair(lower_limit, upper_limit);
    }
    control_action_server_node->set_joint_limits(joint_limits);

    // set joint names
    std::vector<std::string> joint_names;
    for (int j = 0; j < mj_model->njnt; j++) // loop over all joints
    {
        std::string jname = mj_model->names + mj_model->name_jntadr[j];
        joint_names.push_back(jname);
    }
    control_action_server_node->set_joint_names(joint_names);

    rclcpp::spin(control_action_server_node);
    rclcpp::shutdown();
    return 0;
}