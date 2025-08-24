#include <control_actions/control_actions_server.hpp>

namespace control_actions_cpp
{
    ControlActionServer::ControlActionServer(const rclcpp::NodeOptions &options)
        : Node("control_action_server", options)
    {
        this->declare_parameter<std::string>("mujoco_model_path", ""); 
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                for (size_t i = 0; i < msg->name.size(); ++i) {
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

    rclcpp_action::GoalResponse ControlActionServer::move_one_joint_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ControlActions::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with: %s , target position: %f", goal->joint_name.c_str(), goal->target_position);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ControlActionServer::move_one_joint_handle_cancel(const std::shared_ptr<GoalHandleControlActions> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void ControlActionServer::move_one_joint_handle_accepted(const std::shared_ptr<GoalHandleControlActions> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ControlActionServer::move_one_joint_execute, this, _1), goal_handle}.detach();
    }
    std::unordered_map<std::string, double> ControlActionServer::get_current_joint_positions(std::vector<std::string> joint_names)
    {
        return current_joint_positions_;
    }
    void ControlActionServer::move_one_joint_execute(const std::shared_ptr<GoalHandleControlActions> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();                    // Get the user provided message describing the goal.
        auto feedback = std::make_shared<ControlActions::Feedback>(); // The feedback message defined in the action definition.
        // std::shared_ptr<const ControlActions::Goal> goal;
        feedback->joint_name = goal->joint_name;
        feedback->feedback_position = 0.0; // init feedback_position
        auto result = std::make_shared<ControlActions::Result>();

        // validate the joint name
        int joint_id = mj_name2id(mj_model_, mjOBJ_JOINT, goal->joint_name.c_str());
        if (joint_id == -1) // invalid input joint name
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid joint name: %s", goal->joint_name);
            result->joint_name = goal->joint_name;
            result->final_position = NAN;
            return;
        }
        else // valid joint name
        {
            //check the duration value
            if (goal->duration <0.5 || goal ->duration > 5.0)
            {
                RCLCPP_WARN(this->get_logger(), "The duration value is out of range [0.5, 5.0]");
                // cancle the goal
                return;
            }

            // check the target position within joint limits
            if (joint_limits_.find(goal->joint_name) != joint_limits_.end())
            {
                double lower_limit = joint_limits_[goal->joint_name].first;
                double upper_limit = joint_limits_[goal->joint_name].second;
                if (goal->target_position < lower_limit || goal->target_position > upper_limit)
                {
                    RCLCPP_ERROR(this->get_logger(), "Target position %f is out of limits [%f, %f] for joint %s",
                                 goal->target_position, lower_limit, upper_limit, goal->joint_name.c_str());
                    result->joint_name = goal->joint_name;
                    result->final_position = NAN;
                    return; // cancel the goal
                }
            }
            // get the current joint position
            std::unordered_map<std::string, double> target_position = get_current_joint_positions(joint_names_);
            target_position[goal->joint_name] = goal->target_position;

            // create a publisher to publish to the joint trajectory command
            auto traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/joint_trajectory_controller/joint_trajectory", 10);
            // prepare the trajectory message
            trajectory_msgs::msg::JointTrajectory traj_msg;
            traj_msg.joint_names = joint_names_;
            trajectory_msgs::msg::JointTrajectoryPoint point;
            for (const auto &jname : joint_names_)
            {
                point.positions.push_back(target_position[jname]);
            }
            point.time_from_start = rclcpp::Duration::from_seconds(goal->duration);
            traj_msg.points.push_back(point);

            // publish the trajectory command
            traj_pub->publish(traj_msg);
            RCLCPP_INFO(this->get_logger(), "Published trajectory command to move joint %s to position %f in duration %f",
                        goal->joint_name.c_str(), goal->target_position, goal->duration);
            return;
        }
    }
}