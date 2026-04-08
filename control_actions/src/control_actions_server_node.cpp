#include <control_actions/control_actions_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto control_action_server_node = std::make_shared<control_actions::ControlActionServer>();

    // Get robot_description parameter
    std::string robot_description;
    if (!control_action_server_node->get_parameter("robot_description", robot_description))
    {
        RCLCPP_WARN(control_action_server_node->get_logger(), "robot_description parameter not found, trying to get from /robot_description topic...");
        
        // Try to wait for robot_description on global parameter server
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(control_action_server_node, "/robot_state_publisher");
        if (parameters_client->wait_for_service(std::chrono::seconds(5)))
        {
            auto params = parameters_client->get_parameters({"robot_description"});
            if (!params.empty() && params[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                robot_description = params[0].as_string();
            }
        }
        
        if (robot_description.empty())
        {
            RCLCPP_ERROR(control_action_server_node->get_logger(), "Could not get robot_description parameter");
            return 1;
        }
    }

    // Parse URDF
    urdf::Model robot_model;
    if (!robot_model.initString(robot_description))
    {
        RCLCPP_ERROR(control_action_server_node->get_logger(), "Failed to parse URDF");
        return 1;
    }

    RCLCPP_INFO(control_action_server_node->get_logger(), "Successfully parsed URDF model: %s", robot_model.getName().c_str());

    // Extract joint information from URDF
    std::unordered_map<std::string, std::pair<double, double>> joint_limits;
    std::vector<std::string> joint_names;

    for (const auto& joint_pair : robot_model.joints_)
    {
        const auto& joint = joint_pair.second;
        
        // Only process revolute and continuous joints, skip fixed and root joints
        if ((joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) &&
            joint->name != "root")
        {
            joint_names.push_back(joint->name);
            
            if (joint->limits)
            {
                double lower = joint->limits->lower;
                double upper = joint->limits->upper;
                joint_limits[joint->name] = std::make_pair(lower, upper);
                RCLCPP_DEBUG(control_action_server_node->get_logger(), 
                    "Joint %s: limits [%f, %f]", joint->name.c_str(), lower, upper);
            }
            else
            {
                // For continuous joints without limits, use a large range
                joint_limits[joint->name] = std::make_pair(-3.14159, 3.14159);
            }
        }
    }

    RCLCPP_INFO(control_action_server_node->get_logger(), 
        "Loaded %zu joints from URDF", joint_names.size());

    // Set joint information in the node
    control_action_server_node->set_joint_limits(joint_limits);
    control_action_server_node->set_joint_names(joint_names);

    rclcpp::spin(control_action_server_node);
    rclcpp::shutdown();
    return 0;
}