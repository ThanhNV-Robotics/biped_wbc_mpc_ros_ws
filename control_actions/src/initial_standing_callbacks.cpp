#include <control_actions/control_actions_server.hpp>
#include <control_actions/computation_utils.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <thread>
#include <cmath>

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
                    // create VectorXd current_joint_angle from std::vector<double> current_joint_angle;
                    Eigen::VectorXd current_joint_angle_eigen_vector = Eigen::Map<Eigen::VectorXd>(current_joint_angle.data(), current_joint_angle.size());

                    
                    // Extract to Eigen::VectorXd left_leg_current_joint_angles and right leg
                    Eigen::VectorXd left_leg_current_joint_angles = current_joint_angle_eigen_vector.head(6);
                    Eigen::VectorXd right_leg_current_joint_angles = current_joint_angle_eigen_vector.tail(6);

                    // init foot position and orientation
                    Eigen::Vector3d p_foot(0.0, 0.0, -hip_height); // foot position w.r.t the hip
                    Eigen::Matrix3d R_foot = Matrix3d::Identity(); // foot orientation, parallel to the ground

                    // Compute Inverse Kinematics
                    Eigen::VectorXd left_leg_target_joint_angles = control_actions::InvHip2Foot(R_foot, p_foot, left_leg_current_joint_angles, false);  // left leg
                    Eigen::VectorXd right_leg_target_joint_angles = control_actions::InvHip2Foot(R_foot, p_foot, right_leg_current_joint_angles, true); // right leg

                    // Stack left and right leg joint angles into one vector, left leg first
                    VectorXd target_joint_angles;
                    target_joint_angles.resize(left_leg_target_joint_angles.size() +
                           right_leg_target_joint_angles.size()); // must resize to a proper length first
                           
                    target_joint_angles << left_leg_target_joint_angles, right_leg_target_joint_angles;

                    // Log computed joint angles for debugging
                    RCLCPP_INFO(this->get_logger(), "Computed target joint angles:");
                    for (size_t i = 0; i < joint_names.size(); ++i)
                    {
                        RCLCPP_INFO(this->get_logger(), "  %s: %.4f rad (%.2f deg)", 
                                    joint_names[i].c_str(), 
                                    target_joint_angles[i],
                                    target_joint_angles[i] * 180.0 / M_PI);
                    }

                    // Store initial and target positions
                    std::vector<double> initial_positions_vec = current_joint_angle;
                    std::vector<double> target_positions_vec(joint_names.size());
                    for (size_t i = 0; i < joint_names.size(); ++i)
                    {
                        target_positions_vec[i] = target_joint_angles[i];
                    }
                    
                    double duration = goal->duration;
                    RCLCPP_INFO(this->get_logger(), "Executing smooth standing trajectory over %.2f seconds", duration);

                    // create a publisher for PD impedance controller
                    auto cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>(
                        "/pd_impedance_controller/commands", 10);
                    
                    // Allow time for publisher to establish connections
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    
                    // Execute 5th-order polynomial trajectory
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
                            RCLCPP_INFO(this->get_logger(), "Standing trajectory canceled");
                            break;
                        }
                        
                        // Compute 5th-order polynomial: s(t) = 6*(t/T)^5 - 15*(t/T)^4 + 10*(t/T)^3
                        double s = t / duration;
                        double smooth_factor = 6.0 * std::pow(s, 5) - 15.0 * std::pow(s, 4) + 10.0 * std::pow(s, 3);
                        
                        // Clamp to [0, 1]
                        smooth_factor = std::max(0.0, std::min(1.0, smooth_factor));
                        
                        // Compute interpolated positions: theta(t) = theta_s + (theta_e - theta_s) * smooth_factor
                        cmd_msg.header.stamp = this->now();
                        for (size_t i = 0; i < joint_names.size(); ++i)
                        {
                            cmd_msg.position[i] = initial_positions_vec[i] + 
                                                  (target_positions_vec[i] - initial_positions_vec[i]) * smooth_factor;
                        }
                        
                        cmd_pub->publish(cmd_msg);
                        loop_rate.sleep();
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "Standing trajectory completed");

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