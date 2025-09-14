#ifndef STANDING_BALANCE_CONTROL_HPP_
#define STANDING_BALANCE_CONTROL_HPP_

#include "computation_utils.hpp"
#include <Eigen/Dense>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <unordered_map>
#include <string>
#include <vector>

namespace control_actions
{
    class StandingBalanceController : public controller_interface::ControllerInterface
    {
        public:
            // those 3 methods must be implemented
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            // optional methods
            controller_interface::CallbackReturn on_init() override;
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;


            StandingBalanceController(uint16_t control_frequency);

            void set_control_frequency (uint16_t control_frequency)
            {
                control_frequency_ = control_frequency;
                dt_ = 1.0 / static_cast<double>(control_frequency_);
                // re-compute system discrete matrix
                int n = Act.rows();
                int m = Bct.cols();

                // Build augmented matrix
                Eigen::MatrixXd M(n + m, n + m);
                M.setZero();
                M.block(0, 0, n, n) = Act;
                M.block(0, n, n, m) = Bct;

                // Compute matrix exponential
                Eigen::MatrixXd Mexp = (M * dt_).exp();

                // Extract Ad and Bd
                Adis_ = Mexp.block(0, 0, n, n);
                Bdis_ = Mexp.block(0, n, n, m);
            }
            Eigen::VectorXd compute_balancing_motion (double zmp_x, double zmp_y, VectorXd joint_fb_angles);
            Eigen::Vector3d  compute_zmp (Vector3d com_pos, Vector3d acceleration);
            Eigen::VectorXd kalman_filt (VectorXd measurement);
            double get_zc() const { return zc; }

        protected:
            std::vector<std::string> joint_names_; // this shoulde be declared in the yaml config file
            std::vector<std::string> command_interface_types_; // this shoulde be declared in the yaml config file
            std::vector<std::string> state_interface_types_; // this shoulde be declared in the yaml config file

            // command and state interface handles
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
                joint_position_command_interface_; // this related to hardware interface, it is used to send commnand to the robot
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
                joint_position_state_interface_;

            std::unordered_map<
                std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
                command_interface_map_ = {
                {"position", &joint_position_command_interface_}
                };

            std::unordered_map<
                std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
                state_interface_map_ = {
                {"position", &joint_position_state_interface_}};


            // state feedback variables used for control loop
            // imu accelleration
            Eigen::Vector3d imu_acceleration_; // x, y, z acceleration in the imu frame
            // joint state is stored as an unordered map
            Eigen::VectorXd joint_positions_; // all joint positions, left leg + right leg
            Eigen::VectorXd left_leg_current_joint_angles_; // left leg joint angles
            Eigen::VectorXd right_leg_current_joint_angles_; // right leg joint angles

            // IMU subscriber
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
            // joint_state subscriber:
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

            // Controller parameters
            const double Tzmp = 0.01; // sec, time const of the ZMP feedback
            const double zc = 0.62;   // m, CoM height ~ pelvis height
            const double g = 9.81;    // m/s2, gravity const
            Eigen::VectorXd xc_d = Eigen::VectorXd::Zero(4,1); // desired com position and velocity in x and y direction

            uint16_t control_frequency_; // Hz, default value
            double dt_; // sec, control loop period
            // Continuous time state-space matrices
            const Eigen::Matrix<double, 6, 6> Act = (Eigen::Matrix<double, 6, 6>() << -1.0 / Tzmp, 0, 1.0 / Tzmp, 0, 0, 0,
                                            0, -1.0 / Tzmp, 0, 0, 1.0 / Tzmp, 0,
                                            0, 0, 0, 1, 0, 0,
                                            0, 0, 0, 0, 0, 0,
                                            0, 0, 0, 0, 0, 1,
                                            0, 0, 0, 0, 0, 0)
                                                .finished();
            const Eigen::Matrix<double, 6, 2> Bct = (Eigen::Matrix<double, 6, 2>() << -zc / (g * Tzmp), 0,
                                            0, -zc / (g * Tzmp),
                                            0, 0,
                                            1, 0,
                                            0, 0,
                                            0, 1)
                                                .finished();
            // Discrete State Space System
            Eigen::MatrixXd Adis_;
            Eigen::MatrixXd Bdis_;
            // State feedback gain matrix, calculated from python file
            // it depends on Adis, Bdis, Q and R matrix
            // here we dont add Q and R as attributes
            Eigen::Matrix<double, 2, 6> K_ = (Eigen::Matrix<double, 2, 6>() <<
                -3.36492283e+01, 0.0, 9.22916451e+01, 3.08056903e+01, 0.0, 0.0,
                0.0, -3.36600309e+01, 0.0, 0.0, 9.23161487e+01, 3.08127113e+01
            ).finished();

            // Q and R matrices of kalman filter
            // Kalman filter matrices
            const Eigen::MatrixXd A_kalman_ = Eigen::MatrixXd::Identity(3, 3);  // State transition matrix (identity for acceleration filter)
            const Eigen::MatrixXd B_kalman_ = Eigen::MatrixXd::Zero(3, 1);  // Measurement matrix (identity)
            const Eigen::MatrixXd C_kalman_ = Eigen::MatrixXd::Identity(3, 3);
            const Eigen::MatrixXd P_kalman_ =  Eigen::MatrixXd::Identity(3, 3);

            // tuning matrix of Kalman Filter
            const Eigen::MatrixXd R_kalman_ = 2.5 * Eigen::MatrixXd::Identity(3, 3);   // Measurement noise covariance
            const Eigen::MatrixXd Q_kalman_ = 0.02 * Eigen::MatrixXd::Identity(3, 3);  // Process noise covariance
            
            KalmanFilter kalman_filter_ = KalmanFilter(A_kalman_, B_kalman_, C_kalman_, P_kalman_, R_kalman_, Q_kalman_); // init an Kalman filter if needed
    };
} // namespace control_actions
#endif // STANDING_BALANCE_CONTROL_HPP_