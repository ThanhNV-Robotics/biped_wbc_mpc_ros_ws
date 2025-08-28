#include <control_actions/control_actions_server.hpp>

namespace control_actions_cpp
{
    ControlActionServer::ControlActionServer(const rclcpp::NodeOptions &options)
        : Node("control_action_server", options)
    {
        this->declare_parameter<std::string>("mujoco_model_path", "");

        // create a subscription to read joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                for (size_t i = 0; i < msg->name.size(); ++i)
                {
                    current_joint_positions_[msg->name[i]] = msg->position[i];
                }
            }); // read joint position from /joint_states topic
            
        using namespace std::placeholders;
        this->act_serv_move_one_joint_ = rclcpp_action::create_server<ControlActions>(
            this,
            "move_one_joint",
            std::bind(&ControlActionServer::move_one_joint_handle_goal, this, _1, _2),
            std::bind(&ControlActionServer::move_one_joint_handle_cancel, this, _1),
            std::bind(&ControlActionServer::move_one_joint_handle_accepted, this, _1));
    }

    void ControlActionServer::set_mujoco_model(mjModel *mj_model)
    {
        mj_model_ = mj_model;
        return;
    }
    void ControlActionServer::set_joint_limits(std::unordered_map<std::string, std::pair<double, double>> joint_limits)
    {
        joint_limits_ = joint_limits;
        return;
    }
}