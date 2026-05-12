#include <control_actions/control_actions_server.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace control_actions
{
    rclcpp_action::GoalResponse ControlActionServer::left_leg_test_trajectory_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const LeftLegTestTrajectoryAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request for left leg test trajectory with duration: %f", goal->duration);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ControlActionServer::left_leg_test_trajectory_handle_cancel(const std::shared_ptr<GoalHandleLeftLegTestTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel left leg test trajectory goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void ControlActionServer::left_leg_test_trajectory_handle_accepted(const std::shared_ptr<GoalHandleLeftLegTestTrajectory> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ControlActionServer::left_leg_test_trajectory_execute, this, _1), goal_handle}.detach();
    }

    void ControlActionServer::left_leg_test_trajectory_execute(const std::shared_ptr<GoalHandleLeftLegTestTrajectory> goal_handle)
    {
        // get time duration from the action's goal
        double duration = goal_handle->get_goal()->duration;
        if (duration <= 5)
        {
            RCLCPP_INFO(this->get_logger(), "time duration is too short, must be greater than 5 seconds");
            return;
        }
        if (duration > 30)
        {
            RCLCPP_INFO(this->get_logger(), "time duration is too long, must be less than 30 seconds");
            return;
        }

        // create a publisher to publish joint commands to PD impedance controller
        auto cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>(
            "/pd_impedance_controller/commands", 10);

        // Specify joint names
        std::vector<std::string> joint_names = {
            "left_hip_pitch_joint",
            "left_hip_roll_joint", 
            "left_hip_yaw_joint",
            "left_knee_pitch_joint",
            "left_ankle_roll_joint",
            "left_ankle_pitch_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint", 
            "right_hip_yaw_joint",
            "right_knee_pitch_joint",
            "right_ankle_roll_joint",
            "right_ankle_pitch_joint"
        };

        // Joint limits and parameters based on URDF
        std::vector<std::pair<double, double>> joint_limits = {
            {-1.04, 0},    // left_hip_pitch
            {-0.262, 1.05}, // left_hip_roll
            {-0.698, 0.698}, // left_hip_yaw
            {0.0, 1.3},    // left_knee
            {-0.35, 0.35},    // left_ankle_roll
            {-0.5, 0.35}     // left_ankle_pitch
        };

        // Trajectory frequencies for each joint (Hz)
        std::vector<double> frequencies = {0.25, 0.275, 0.3, 0.35, 0.35, 0.45};
        
        // Execute trajectory by publishing commands in real-time
        double init_duration = 2.5; // seconds for initialization
        double dt = 0.004; // 250Hz command rate to match controller frequency
        rclcpp::Rate loop_rate(1.0 / dt); // 250 Hz
        
        // Low-pass filter parameters
        double cutoff_frequency = 2.0; // Hz
        double sample_rate = 1.0 / dt;
        double alpha = (2.0 * M_PI * dt * cutoff_frequency) / (2.0 * M_PI * dt * cutoff_frequency + 1.0);
        std::vector<double> filtered_positions(12, 0.0);
        
        auto feedback = std::make_shared<LeftLegTestTrajectoryAction::Feedback>();
        auto result = std::make_shared<LeftLegTestTrajectoryAction::Result>();
        
        for (double t = 0.0; t < duration; t += dt) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Left leg trajectory was canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Left leg trajectory canceled");
                return;
            }
            
            // Create command message
            sensor_msgs::msg::JointState cmd_msg;
            cmd_msg.header.stamp = this->now();
            cmd_msg.name = joint_names;
            cmd_msg.position.resize(12);
            cmd_msg.velocity.resize(12, 0.0);  // Zero velocity for position tracking
            cmd_msg.effort.resize(12, 0.0);    // Zero feedforward torque
            
            for (size_t j = 0; j < 12; ++j) {
                if (j >= 6) {
                    // Right leg joints remain at zero position
                    cmd_msg.position[j] = 0.0;
                    continue;
                }
                double lower_limit = joint_limits[j].first;
                double upper_limit = joint_limits[j].second;
                double mid_point = (lower_limit + upper_limit) / 2.0;
                double amplitude = 0.8 * (upper_limit - lower_limit) / 2.0;
                
                double raw_position = 0.0;
                
                if (t <= init_duration) {
                    // Smooth initialization phase using 5th order polynomial
                    double s = t / init_duration;
                    double smooth_factor = 6 * std::pow(s, 5) - 15 * std::pow(s, 4) + 10 * std::pow(s, 3);
                    raw_position = smooth_factor * mid_point;
                } else {
                    // Sinusoidal trajectory phase
                    double phase_time = t - init_duration;
                    double omega = 2.0 * M_PI * frequencies[j];
                    raw_position = mid_point + amplitude * std::sin(omega * phase_time);
                }
                
                // Apply first-order low-pass filter
                filtered_positions[j] = alpha * raw_position + (1.0 - alpha) * filtered_positions[j];
                cmd_msg.position[j] = filtered_positions[j];
            }
            
            // Publish command
            cmd_pub->publish(cmd_msg);
            
            // Update feedback every 0.1 seconds
            if (static_cast<int>(t * 10) % static_cast<int>(0.1 / dt * 10) == 0) {
                feedback->elapsed_time = t;
                feedback->remaining_time = duration - t;
                goal_handle->publish_feedback(feedback);
            }
            
            loop_rate.sleep();
        }
        
        RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
        
        // Check if goal is still active and set result
        if (rclcpp::ok() && !goal_handle->is_canceling()) {
            result->success = true;
            result->message = "Left leg trajectory completed successfully";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Left leg trajectory completed successfully");
        }
    }
} // namespace control_actions