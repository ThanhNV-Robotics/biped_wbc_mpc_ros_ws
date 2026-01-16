#include <control_actions/control_actions_server.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace control_actions
{
    rclcpp_action::GoalResponse ControlActionServer::homing_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const HomingAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with target hip height: %f", goal->duration);
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

        // create a publisher to publish 0.0 value to all joints' position
        auto joint_trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        // create a joint trajectory message
        trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
        joint_trajectory_msg.joint_names = joint_names_;
        joint_trajectory_msg.points.resize(1);
        joint_trajectory_msg.points[0].positions.resize(joint_names_.size(), 0.0);
        joint_trajectory_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(duration);

        // publish the joint trajectory message
        joint_trajectory_pub->publish(joint_trajectory_msg);

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