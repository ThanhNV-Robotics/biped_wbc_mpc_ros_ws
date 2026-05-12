#include <control_actions/control_actions_server.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <cmath>

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace control_actions
{
    rclcpp_action::GoalResponse ControlActionServer::homing_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const HomingAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received homing goal request with duration: %f", goal->duration);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ControlActionServer::homing_handle_cancel(const std::shared_ptr<GoalHandleHoming> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void ControlActionServer::homing_handle_accepted(const std::shared_ptr<GoalHandleHoming> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ControlActionServer::homing_execute, this, _1), goal_handle}.detach();
    }

    void ControlActionServer::homing_execute(const std::shared_ptr<GoalHandleHoming> goal_handle)
    {
        // get time duration from the action's goal
        double duration = goal_handle->get_goal()->duration;
        if (duration <= 2)
        {
            RCLCPP_INFO(this->get_logger(), "time duration is too short, must be greater than 2 seconds");
            return;
        }
        if (duration > 10)
        {
            RCLCPP_INFO(this->get_logger(), "time duration is too long, must be less than 10 seconds");
            return;
        }

        // Get joint names
        std::vector<std::string> joint_names = ControlActionServer::get_joint_names();
        if (joint_names.size() != 12)
        {
            RCLCPP_ERROR(this->get_logger(), "Expected 12 joint names, got %zu", joint_names.size());
            return;
        }
        
        // Get current joint positions
        std::unordered_map<std::string, double> current_positions = ControlActionServer::get_current_joint_positions(joint_names);
        std::vector<double> initial_positions(joint_names.size());
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            auto it = current_positions.find(joint_names[i]);
            if (it != current_positions.end())
            {
                initial_positions[i] = it->second;
            }
            else
            {
                initial_positions[i] = 0.0;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Executing smooth homing trajectory over %.2f seconds", duration);

        // create a publisher to publish commands
        auto cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>(
            "/pd_impedance_controller/commands", 10);
        
        // Allow time for publisher to establish connections
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Execute 5th-order polynomial trajectory to zero position
        double dt = 0.004; // 250Hz command rate
        rclcpp::Rate loop_rate(1.0 / dt);
        
        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.name = joint_names;
        cmd_msg.position.resize(joint_names.size());
        cmd_msg.velocity.resize(joint_names.size(), 0.0);
        cmd_msg.effort.resize(joint_names.size(), 0.0);
        
        for (double t = 0.0; t <= duration; t += dt)
        {
            if (goal_handle->is_canceling())
            {
                auto result = std::make_shared<HomingAction::Result>();
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Homing trajectory canceled");
                return;
            }
            
            // Compute 5th-order polynomial: s(t) = 6*(t/T)^5 - 15*(t/T)^4 + 10*(t/T)^3
            double s = t / duration;
            double smooth_factor = 6.0 * std::pow(s, 5) - 15.0 * std::pow(s, 4) + 10.0 * std::pow(s, 3);
            
            // Clamp to [0, 1]
            smooth_factor = std::max(0.0, std::min(1.0, smooth_factor));
            
            // Compute interpolated positions: theta(t) = theta_s + (0 - theta_s) * smooth_factor
            cmd_msg.header.stamp = this->now();
            for (size_t i = 0; i < joint_names.size(); ++i)
            {
                cmd_msg.position[i] = initial_positions[i] * (1.0 - smooth_factor);
            }
            
            cmd_pub->publish(cmd_msg);
            loop_rate.sleep();
        }
        
        RCLCPP_INFO(this->get_logger(), "Homing trajectory completed");

        // Check if goal is done
        if (rclcpp::ok() && !goal_handle->is_canceling())
        {
            auto result = std::make_shared<HomingAction::Result>();
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            return;
        }
    }
}