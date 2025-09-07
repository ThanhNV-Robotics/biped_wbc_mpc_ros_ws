#include <control_actions/control_actions_server.hpp>

namespace control_actions
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

        // create move_one_joint action server
        this->act_serv_move_one_joint_ = rclcpp_action::create_server<MoveOneJointAction>(
            this,
            "move_one_joint",
            std::bind(&ControlActionServer::move_one_joint_handle_goal, this, _1, _2),
            std::bind(&ControlActionServer::move_one_joint_handle_cancel, this, _1),
            std::bind(&ControlActionServer::move_one_joint_handle_accepted, this, _1));
        
        // create initial_standing action server

        this->act_serv_initial_standing_ = rclcpp_action::create_server<InitialStandingAction>(
            this,
            "initial_standing",
            std::bind(&ControlActionServer::initial_standing_handle_goal, this, _1, _2),
            std::bind(&ControlActionServer::initial_standing_handle_cancel, this, _1),
            std::bind(&ControlActionServer::initial_standing_handle_accepted, this, _1));

        // create standing_balance action server
        this->act_serv_standing_balance_ = rclcpp_action::create_server<StandingBalanceAction>(
            this,
            "standing_balance",
            std::bind(&ControlActionServer::standing_balance_handle_goal, this, _1, _2),
            std::bind(&ControlActionServer::standing_balance_handle_cancel, this, _1),
            std::bind(&ControlActionServer::standing_balance_handle_accepted, this, _1));

        // init standing balancing controller
        // Retrieve control frequency from controller manager
        try
        {
            // Create a temporary node to query the parameter
            auto temp_node = std::make_shared<rclcpp::Node>("temp_control_actions_node");
            control_frequency_ = temp_node->get_parameter("/controller_manager/update_rate").as_int();
            RCLCPP_INFO(this->get_logger(), "Retrieved control frequency from controller manager: %d Hz", control_frequency_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to retrieve control frequency from controller manager: %s. Using default 250 Hz.", e.what());
            control_frequency_ = 250;  // Default value
        }
        // Initialize standing balance controller with the retrieved control frequency
        standing_controller_.set_control_frequency(control_frequency_);
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

    std::unordered_map<std::string, double> ControlActionServer::get_current_joint_positions(std::vector<std::string> joint_names)
    {
        return current_joint_positions_;
    }
}