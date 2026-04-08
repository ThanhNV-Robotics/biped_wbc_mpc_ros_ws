#include <control_actions/control_actions_server.hpp>

namespace control_actions
{
    ControlActionServer::ControlActionServer(const rclcpp::NodeOptions &options)
        : Node("control_action_server", options)
    {
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
        
        // create a subsription to read imu data
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_sensor_broadcaster/imu", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                imu_data_["angular_velocity_x"] = msg->angular_velocity.x;
                imu_data_["angular_velocity_y"] = msg->angular_velocity.y;
                imu_data_["angular_velocity_z"] = msg->angular_velocity.z;
                imu_data_["linear_acceleration_x"] = msg->linear_acceleration.x;
                imu_data_["linear_acceleration_y"] = msg->linear_acceleration.y;
                imu_data_["linear_acceleration_z"] = msg->linear_acceleration.z;
            }); // read imu data from /imu/data topic
            
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

        // create homing action server
        this->act_serv_homing_ = rclcpp_action::create_server<HomingAction>(
            this,
            "homing",
            std::bind(&ControlActionServer::homing_handle_goal, this, _1, _2),
            std::bind(&ControlActionServer::homing_handle_cancel, this, _1),
            std::bind(&ControlActionServer::homing_handle_accepted, this, _1));

        // create left_leg_test_trajectory action server
        this->act_serv_left_leg_test_trajectory_ = rclcpp_action::create_server<LeftLegTestTrajectoryAction>(
            this,
            "left_leg_test_trajectory",
            std::bind(&ControlActionServer::left_leg_test_trajectory_handle_goal, this, _1, _2),
            std::bind(&ControlActionServer::left_leg_test_trajectory_handle_cancel, this, _1),
            std::bind(&ControlActionServer::left_leg_test_trajectory_handle_accepted, this, _1));
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