#include <control_actions/control_actions_server.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
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

        // create a publisher to publish joint trajectory to left leg joints
        auto joint_trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        // create a joint trajectory message
        trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
        // specify left leg joint names
        joint_trajectory_msg.joint_names = {
            "left_hip_pitch",
            "left_hip_roll", 
            "left_hip_yaw",
            "left_knee",
            "left_ankle_roll",
            "left_ankle_pitch",
            "right_hip_pitch",
            "right_hip_roll", 
            "right_hip_yaw",
            "right_knee",
            "right_ankle_roll",
            "right_ankle_pitch"
        };

        // Joint limits and parameters based on URDF
        std::vector<std::pair<double, double>> joint_limits = {
            {-1.4, 1.4},    // left_hip_pitch
            {-0.262, 1.05}, // left_hip_roll
            {-0.698, 0.698}, // left_hip_yaw
            {0.0, 2.09},    // left_knee
            {-0.5, 0.5},    // left_ankle_roll
            {-0.5, 0.5}     // left_ankle_pitch
        };

        // Trajectory frequencies for each joint (Hz)
        std::vector<double> frequencies = {0.35, 0.25, 0.3, 0.35, 0.35, 0.45};
        
        // Generate trajectory points
        double init_duration = 2.0; // seconds for initialization
        double trajectory_duration = duration - init_duration;
        double dt = 0.004; // 250Hz trajectory points to match controller frequency
        int num_points = static_cast<int>(duration / dt);
        
        // Low-pass filter parameters
        double cutoff_frequency = 3.0; // Hz - reduce this (try 2.0-3.0) for smoother motion
        double sample_rate = 1.0 / dt; // 250 Hz
        double alpha = (2.0 * M_PI * dt * cutoff_frequency) / (2.0 * M_PI * dt * cutoff_frequency + 1.0);
        std::vector<double> filtered_positions(12, 0.0); // Store previous filtered values
        
        joint_trajectory_msg.points.resize(num_points);
        
        for (int i = 0; i < num_points; ++i) {
            double t = i * dt;
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions.resize(12);
            // point.velocities.resize(12);
            // point.accelerations.resize(12);
            
            for (size_t j = 0; j < 12; ++j) {
                if (j >= 6) {
                    // Right leg joints remain at zero position
                    point.positions[j] = 0.0;
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
                    // point.velocities[j] = (30 * std::pow(t, 4) / std::pow(init_duration, 5) - 
                    //                      60 * std::pow(t, 3) / std::pow(init_duration, 4) + 
                    //                      30 * std::pow(t, 2) / std::pow(init_duration, 3)) * mid_point;
                    // point.accelerations[j] = (120 * std::pow(t, 3) / std::pow(init_duration, 5) - 
                                            // 180 * std::pow(t, 2) / std::pow(init_duration, 4) + 
                                            // 60 * t / std::pow(init_duration, 3)) * mid_point;
                } else if (t <= duration) {
                    // Sinusoidal trajectory phase
                    double phase_time = t - init_duration;
                    double omega = 2.0 * M_PI * frequencies[j];
                    
                    raw_position = mid_point + amplitude * std::sin(omega * phase_time);
                    // point.velocities[j] = amplitude * omega * std::cos(omega * phase_time);
                    // point.accelerations[j] = -amplitude * omega * omega * std::sin(omega * phase_time);
                } else {
                    // Hold final position
                    raw_position = mid_point;
                    // point.velocities[j] = 0.0;
                    // point.accelerations[j] = 0.0;
                }
                
                // Apply first-order low-pass filter
                filtered_positions[j] = alpha * raw_position + (1.0 - alpha) * filtered_positions[j];
                point.positions[j] = filtered_positions[j];
            }
            
            point.time_from_start = rclcpp::Duration::from_seconds(t);
            joint_trajectory_msg.points[i] = point;
        }
        
        // Publish trajectory
        joint_trajectory_pub->publish(joint_trajectory_msg);
        RCLCPP_INFO(this->get_logger(), "Trajectory published successfully");
        
        // Wait for trajectory execution and provide feedback
        rclcpp::Rate loop_rate(10); // 10 Hz feedback rate
        auto feedback = std::make_shared<LeftLegTestTrajectoryAction::Feedback>();
        auto result = std::make_shared<LeftLegTestTrajectoryAction::Result>();
        
        for (double elapsed_time = 0.0; elapsed_time < duration; elapsed_time += 0.1) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Left leg trajectory was canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Left leg trajectory canceled");
                return;
            }
            
            // Update feedback
            feedback->elapsed_time = elapsed_time;
            feedback->remaining_time = duration - elapsed_time;
            goal_handle->publish_feedback(feedback);
            
            loop_rate.sleep();
        }
        
        // Check if goal is still active and set result
        if (rclcpp::ok() && !goal_handle->is_canceling()) {
            result->success = true;
            result->message = "Left leg trajectory completed successfully";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Left leg trajectory completed successfully");
        }
    }
} // namespace control_actions