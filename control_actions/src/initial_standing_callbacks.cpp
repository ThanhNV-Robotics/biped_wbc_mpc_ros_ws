#include <control_actions/control_actions_server.hpp>
#include <control_actions/computation_utils.hpp>
#include <Eigen/Dense>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace control_actions
{
    rclcpp_action::GoalResponse ControlActionServer::initial_standing_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const InitialStandingAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with target hip height: %f", goal->hip_height);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ControlActionServer::initial_standing_handle_cancel(const std::shared_ptr<GoalHandleInitialStanding> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void ControlActionServer::initial_standing_handle_accepted(const std::shared_ptr<GoalHandleInitialStanding> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ControlActionServer::initial_standing_execute, this, _1), goal_handle}.detach();
    }

    void ControlActionServer::initial_standing_execute(const std::shared_ptr<GoalHandleInitialStanding> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal(); // Get the user provided message describing the goal.

        // validate the hip height within the limit
        double hip_height = goal->hip_height;
        if (hip_height < 0.59 || hip_height > 0.65)
        {
            RCLCPP_ERROR(this->get_logger(), "The target hip height is out of range [0.59, 0.65]");
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "The target hip height is valid: %f", hip_height);

            // get joint name
            std::vector<std::string> joint_names =  ControlActionServer::get_joint_names();
            if (joint_names.size() != 12) // must have enough both legs
            {
                RCLCPP_ERROR(this->get_logger(), "Expected 12 joint names, got %zu", joint_names.size());
                return;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Got %zu joint names", joint_names.size());
                // get current joint position
                std::unordered_map<std::string, double> current_joint_positions =  ControlActionServer::get_current_joint_positions(joint_names);
                std::vector<double> current_joint_angle;
                for (const auto &name : joint_names)
                {
                    auto it = current_joint_positions_.find(name);
                    if (it != current_joint_positions_.end())
                    {
                        current_joint_angle.push_back(it->second);
                    }
                    else
                    {
                        current_joint_angle.push_back(0.0); // or NAN, or handle as needed
                    }
                }

                if (current_joint_angle.size() != 12)
                {
                    RCLCPP_ERROR(this->get_logger(), "Expected 12 current joint angles, got %zu", current_joint_angle.size());
                    return;
                }
                else
                {
                    // Assume current_joint_angle has at least 12 elements
                    std::vector<double> left_leg_current_joint_angles(current_joint_angle.begin(), current_joint_angle.begin() + 6);
                    std::vector<double> right_leg_current_joint_angles(current_joint_angle.end() - 6, current_joint_angle.end());

                    // init foot position and orientation
                    Vector3d p_foot(0.0, 0.0, -hip_height); // foot position w.r.t the hip
                    Matrix3d R_foot = Matrix3d::Identity(); // foot orientation, parallel to the ground

                    // Compute Inverse Kinematics
                    std::vector<double> left_leg_target_joint_angles = control_actions::InvHip2Foot(R_foot, p_foot, left_leg_current_joint_angles, false);  // left leg
                    std::vector<double> right_leg_target_joint_angles = control_actions::InvHip2Foot(R_foot, p_foot, right_leg_current_joint_angles, true); // right leg

                    // Stack left and right leg joint angles into one vector, left leg first
                    std::vector<double> target_joint_angles;
                    target_joint_angles.insert(target_joint_angles.end(),
                                               left_leg_target_joint_angles.begin(), left_leg_target_joint_angles.end());
                    target_joint_angles.insert(target_joint_angles.end(),
                                               right_leg_target_joint_angles.begin(), right_leg_target_joint_angles.end());

                    // create publishing message to joint trajectory command
                    // created target_joint_map
                    std::unordered_map<std::string, double> target_joint_map;
                    for (size_t i = 0; i < joint_names.size(); ++i)
                    {
                        target_joint_map[joint_names[i]] = target_joint_angles[i];
                    }

                    // prepare the trajectory message
                    trajectory_msgs::msg::JointTrajectory traj_msg;
                    traj_msg.joint_names = joint_names;
                    trajectory_msgs::msg::JointTrajectoryPoint point;

                    for (const auto &name : joint_names)
                    {
                        point.positions.push_back(target_joint_map[name]);
                    }

                    point.time_from_start = rclcpp::Duration::from_seconds(goal->duration); // duration to reach the target
                    traj_msg.points.push_back(point);
                    auto traj_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                        "/joint_trajectory_controller/joint_trajectory", 10);
                    traj_pub->publish(traj_msg);

                    // publish feedback
                    auto feedback = std::make_shared<InitialStandingAction::Feedback>();
                    feedback->structure_needs_at_least_one_member = 1; // dummy feedback
                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Published feedback");

                    // Check if goal is done
                    if (rclcpp::ok())
                    {
                        auto result = std::make_shared<InitialStandingAction::Result>();
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                    }
                }
            }
        }
    }
}