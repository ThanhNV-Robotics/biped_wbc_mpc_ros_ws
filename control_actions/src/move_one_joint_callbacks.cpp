#include <control_actions/control_actions_server.hpp>

namespace control_actions
{
    rclcpp_action::GoalResponse ControlActionServer::move_one_joint_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveOneJointAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with: %s , target position: %f", goal->joint_name.c_str(), goal->target_position);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ControlActionServer::move_one_joint_handle_cancel(const std::shared_ptr<GoalHandleCMoveOneJoint> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void ControlActionServer::move_one_joint_handle_accepted(const std::shared_ptr<GoalHandleCMoveOneJoint> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ControlActionServer::move_one_joint_execute, this, _1), goal_handle}.detach();
    }

    void ControlActionServer::move_one_joint_execute(const std::shared_ptr<GoalHandleCMoveOneJoint> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();                    // Get the user provided message describing the goal.
        auto feedback = std::make_shared<MoveOneJointAction::Feedback>(); // The feedback message defined in the action definition.
        // std::shared_ptr<const MoveOneJointAction::Goal> goal;
        feedback->joint_name = goal->joint_name;
        feedback->feedback_position = 0.0; // init feedback_position
        auto result = std::make_shared<MoveOneJointAction::Result>();

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
            // check the duration value
            if (goal->duration < 0.5 || goal->duration > 5.0)
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
            std::vector<std::string> joint_names =  ControlActionServer::get_joint_names();
            // get the current joint position
            std::unordered_map<std::string, double> target_position = ControlActionServer::get_current_joint_positions(joint_names);
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
            // Feedback loop, update feedback at 20 Hz
            // update current joint positions
            rclcpp::Rate rate(20); // 20 Hz feedback
            while (rclcpp::ok())
            {
                double current_position = 0.0;
                //current_joint_positions_ subscribes to /joint_states
                auto it = current_joint_positions_.find(goal->joint_name);
                if (it != current_joint_positions_.end())
                    current_position = it->second;

                feedback->joint_name = goal->joint_name;
                feedback->feedback_position = current_position;
                goal_handle->publish_feedback(feedback);

                double error = std::abs(current_position - goal->target_position);
                if (error <= 0.0872665) // 5 degrees in radians
                {
                    result->joint_name = goal->joint_name;
                    result->final_position = current_position;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded for joint %s at position %f (error: %f rad)",
                                goal->joint_name.c_str(), current_position, error);
                    break;
                }

                if (!rclcpp::ok() || goal_handle->is_canceling())
                {
                    result->joint_name = goal->joint_name;
                    result->final_position = current_position;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled for joint %s", goal->joint_name.c_str());
                    break;
                }

                rate.sleep();
            }
        }
    }
}