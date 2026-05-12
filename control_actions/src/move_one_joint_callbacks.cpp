#include <control_actions/control_actions_server.hpp>
#include <chrono>
#include <thread>
#include <cmath>

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
        auto joint_names = this->get_joint_names();
        bool joint_found = std::find(joint_names.begin(), joint_names.end(), goal->joint_name) != joint_names.end();
        
        if (!joint_found) // invalid input joint name
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid joint name: %s", goal->joint_name.c_str());
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
            // get the current joint positions
            std::unordered_map<std::string, double> initial_positions = ControlActionServer::get_current_joint_positions(joint_names);
            
            // Store initial and target positions for all joints
            std::unordered_map<std::string, double> target_positions = initial_positions;
            target_positions[goal->joint_name] = goal->target_position;
            
            double initial_pos = initial_positions[goal->joint_name];
            double target_pos = goal->target_position;
            double duration = goal->duration;

            // create a publisher to publish to the PD impedance controller
            auto cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>(
                "/pd_impedance_controller/commands", 10);
            
            // Allow time for publisher to establish connections
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            RCLCPP_INFO(this->get_logger(), "Executing smooth trajectory for %s from %.3f to %.3f over %.2f seconds",
                        goal->joint_name.c_str(), initial_pos, target_pos, duration);
            
            // Execute 5th-order polynomial trajectory
            double dt = 0.004; // 250Hz command rate
            rclcpp::Rate loop_rate(1.0 / dt);
            
            sensor_msgs::msg::JointState cmd_msg;
            cmd_msg.name = joint_names_;
            cmd_msg.position.resize(joint_names_.size());
            cmd_msg.velocity.resize(joint_names_.size(), 0.0);
            cmd_msg.effort.resize(joint_names_.size(), 0.0);
            
            for (double t = 0.0; t <= duration; t += dt)
            {
                if (goal_handle->is_canceling())
                {
                    RCLCPP_INFO(this->get_logger(), "Trajectory canceled");
                    break;
                }
                
                // Compute 5th-order polynomial: s(t) = 6*(t/T)^5 - 15*(t/T)^4 + 10*(t/T)^3
                double s = t / duration;
                double smooth_factor = 6.0 * std::pow(s, 5) - 15.0 * std::pow(s, 4) + 10.0 * std::pow(s, 3);
                
                // Clamp to [0, 1]
                smooth_factor = std::max(0.0, std::min(1.0, smooth_factor));
                
                // Compute interpolated position: theta(t) = theta_s + (theta_e - theta_s) * smooth_factor
                double interpolated_pos = initial_pos + (target_pos - initial_pos) * smooth_factor;
                
                // Set positions for all joints (only target joint moves)
                cmd_msg.header.stamp = this->now();
                for (size_t i = 0; i < joint_names_.size(); ++i)
                {
                    if (joint_names_[i] == goal->joint_name)
                    {
                        cmd_msg.position[i] = interpolated_pos;
                    }
                    else
                    {
                        cmd_msg.position[i] = initial_positions[joint_names_[i]];
                    }
                }
                
                cmd_pub->publish(cmd_msg);
                loop_rate.sleep();
            }
            
            RCLCPP_INFO(this->get_logger(), "Trajectory execution completed");
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