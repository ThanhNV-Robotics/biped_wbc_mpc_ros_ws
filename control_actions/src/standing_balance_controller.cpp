#include "control_actions/standing_balance_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace control_actions
{
    // default constructor
    StandingBalanceController::StandingBalanceController() : controller_interface::ControllerInterface() {}

    std::string StandingBalanceController::get_robot_description()
    {
        // Getting robot description from parameter first. If not set trying from topic
        std::string robot_description;

        auto node = std::make_shared<rclcpp::Node>(
            "robot_description_node",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

        if (node->has_parameter("robot_description"))
        {
            robot_description = node->get_parameter("robot_description").as_string();
            return robot_description;
        }

        RCLCPP_INFO(node->get_logger(), "Failed to get robot_description from parameter. Will listen on the ~/robot_description topic...");

        auto robot_description_sub = node->create_subscription<std_msgs::msg::String>(
            "robot_description", rclcpp::QoS(1).transient_local(),
            [&](const std_msgs::msg::String::SharedPtr msg)
            {
                if (!msg->data.empty() && robot_description.empty())
                    robot_description = msg->data;
            });

        while (robot_description.empty() && rclcpp::ok())
        {
            rclcpp::spin_some(node);
            RCLCPP_INFO(node->get_logger(), "Waiting for robot description message");
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        return robot_description;
    }
    controller_interface::CallbackReturn StandingBalanceController::on_init()
    {
        // get controller's update_rate from the controller manager
        control_frequency_ = auto_declare<int>("update_rate", control_frequency_);

        // check valid condition
        if (control_frequency_ <= 0)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Invalid control frequency: %d", control_frequency_);
            return CallbackReturn::FAILURE;
        }

        // init the controller with a specified control frequency
        StandingBalanceController::set_control_frequency(control_frequency_);
        RCLCPP_INFO(get_node()->get_logger(), "StandingBalanceController will run at %d Hz", control_frequency_);

        // get joint names from the yaml config file
        // should have error handling
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_); // set joint name from the config yaml file
        command_interface_types_ =
            auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_); // set command interface type from the config yaml file
        state_interface_types_ =
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_); // set state interface type from the config yaml file

        // get mujoco xml model from the parameter
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
            "standing_balance_controller_node",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
        RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
        auto model_path = node->get_parameter("mujoco_model_path").as_string();
        // Get file name from model_path
        std::string file_name_str = model_path.substr(model_path.find_last_of("/\\") + 1);
        const char *file_name = file_name_str.c_str();
        RCLCPP_INFO_STREAM(node->get_logger(), "File name: " << file_name);
        // load and compile model
        char error[1000] = "Could not load binary model";
        if (
            std::strlen(model_path.c_str()) > 4 &&
            !std::strcmp(model_path.c_str() + std::strlen(model_path.c_str()) - 4, ".mjb"))
        {
            mj_model_ = mj_loadModel(model_path.c_str(), 0);
        }
        else
        {
            mj_model_ = mj_loadXML(model_path.c_str(), 0, error, 1000);
        }
        if (!mj_model_)
        {
            mju_error("Load model error: %s", error);
        }

        RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
        // make data
        mj_data_ = mj_makeData(mj_model_);

        // init size of joint_positions_ according to the number of joint names
        if (joint_names_.size() != 12) // we expect 12 joints for the biped robot, otherwise error
        {
            RCLCPP_ERROR(get_node()->get_logger(), "No joint names provided to the StandingBalanceController");
            return CallbackReturn::FAILURE;
        }
        joint_positions_.resize(joint_names_.size());
        left_leg_current_joint_angles_.resize(6);  // 6 DoF for left leg
        right_leg_current_joint_angles_.resize(6); // 6 DoF for right leg
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn control_actions::StandingBalanceController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Add any cleanup or deactivation logic here if needed
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn StandingBalanceController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        // init the imu_subscriber_
        imu_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_sensor_broadcaster/imu", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                // callback function to process the incoming IMU data
                // set to imu_acceleration_
                imu_acceleration_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
            });
        // init the joint_state_subscriber_
        joint_state_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                // set to joint_positions_ in order of joint_names_
                for (size_t i = 0; i < joint_names_.size(); ++i)
                {
                    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                    if (it != msg->name.end())
                    {
                        size_t index = std::distance(msg->name.begin(), it);
                        joint_positions_[i] = msg->position[index];
                    }
                }
                // extract left and right leg joint angles
                left_leg_current_joint_angles_ = joint_positions_.head(6);  // first 6 joints are left leg
                right_leg_current_joint_angles_ = joint_positions_.tail(6); // last 6 joints are right leg
            });

        // for testing, init a publisher to publish kalman filter output
        kalman_filter_publisher_ = get_node()->create_publisher<sensor_msgs::msg::Imu>(
            "/controller_kalman_filter_output", rclcpp::QoS(10));

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn StandingBalanceController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        // currently do nothing
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration StandingBalanceController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : command_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration StandingBalanceController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    // update method -> main control loop is run here
    controller_interface::return_type StandingBalanceController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // filter the imu_acceleration_ using kalman filter
        // filt acceleration data and publish it
        Eigen::VectorXd filtered_acceleration = StandingBalanceController::kalman_filt(imu_acceleration_);
        sensor_msgs::msg::Imu filtered_acc_msg;
        filtered_acc_msg.linear_acceleration.x = filtered_acceleration[0];
        filtered_acc_msg.linear_acceleration.y = filtered_acceleration[1];
        filtered_acc_msg.linear_acceleration.z = filtered_acceleration[2];
        if (kalman_filter_publisher_)
        {
            kalman_filter_publisher_->publish(filtered_acc_msg);
        }
    }

    StandingBalanceController::StandingBalanceController(uint16_t control_frequency)
    {
        control_frequency_ = control_frequency;
        dt_ = 1.0 / control_frequency_;

        // compute discete system matrix
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
    Eigen::VectorXd StandingBalanceController::compute_balancing_motion(double zmp_x, double zmp_y, VectorXd joint_fb_angles)
    {
        // extract left_leg_current_joint_angles and right angles from the joint feedback angles
        Eigen::VectorXd left_leg_current_joint_angles = joint_fb_angles.head(6);
        Eigen::VectorXd right_leg_current_joint_angles = joint_fb_angles.tail(6);

        // stack zmp_x and zmp_y to (2,1) vector
        Eigen::VectorXd zmp(2);
        zmp << zmp_x, zmp_y;

        // stack x_state = np.vstack((zmp_fb, self.xc_d))
        Eigen::VectorXd x_state(4);
        x_state << zmp, xc_d;

        // compute control input u = -self.K@x_state
        Eigen::VectorXd u = -K_ * x_state;

        // calculate next time step state x_update = self.Adis@x_state  + self.Bdis@u
        Eigen::VectorXd x_update = Adis_ * x_state + Bdis_ * u;

        // update current state self.xc_d = x_update[2:]
        xc_d = x_update.tail(2);

        // extract com's desired position com_x = x_update[2].item()
        double com_x = x_update[2];
        double com_y = x_update[3];

        // compute inverse kinematics
        // init foot position and orientation
        Vector3d p_foot(0.0, 0.0, -zc);         // foot position w.r.t the hip, must pass zc as the hip_height constrain
        Matrix3d R_foot = Matrix3d::Identity(); // foot orientation, parallel to the ground

        // Compute Inverse Kinematics
        Eigen::VectorXd left_leg_target_joint_angles = control_actions::InvHip2Foot(R_foot, p_foot, left_leg_current_joint_angles, false);  // left leg
        Eigen::VectorXd right_leg_target_joint_angles = control_actions::InvHip2Foot(R_foot, p_foot, right_leg_current_joint_angles, true); // right leg

        // Stack left and right leg joint angles into one vector, left leg first
        Eigen::VectorXd target_joint_angles;
        target_joint_angles.resize(left_leg_target_joint_angles.size() +
                                   right_leg_target_joint_angles.size()); // must resize to a proper length first

        target_joint_angles << left_leg_target_joint_angles, right_leg_target_joint_angles;
        return target_joint_angles;
    }
    Eigen::Vector3d StandingBalanceController::compute_zmp(Vector3d com_pos, Vector3d acceleration)
    {
        double x = com_pos[0];
        double y = com_pos[1];
        double z = com_pos[2];

        VectorXd filted_acceleration = kalman_filt(acceleration);
        double acc_x = filted_acceleration[0];
        double acc_y = filted_acceleration[1];
        double acc_z = filted_acceleration[2];

        // double px = x-(z*acc_x)/g; // unit in m/s2
        // double py = y-(z*acc_y)/g; // unit in m/s2

        double px = x - (z * acc_x); // unit in g
        double py = y - (z * acc_y); // unit in g

        double pz = 0.0;

        Vector3d zmp;
        zmp << px, py, pz;
        return zmp;
    }
    Eigen::VectorXd StandingBalanceController::kalman_filt(VectorXd measurement)
    {
        return kalman_filter_.kalman_filt(measurement);
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    control_actions::StandingBalanceController, controller_interface::ControllerInterface)