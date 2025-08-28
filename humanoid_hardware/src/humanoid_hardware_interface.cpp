#include "humanoid_hardware/humanoid_hardware_interface.hpp"
#include "humanoid_hardware/ankle_kinematics.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace humanoid_hardware
{

hardware_interface::CallbackReturn HumanoidHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"), "Initializing...");

    size_t num_joints = info_.joints.size();
    hw_commands_.resize(num_joints, 0.0);
    hw_positions_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
    hw_torques_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
    joint_map_.resize(num_joints);

    for (uint i = 0; i < num_joints; i++)
    {
        joint_map_[i].name = info_.joints[i].name;

        if (info_.joints[i].parameters.find("motor_id") == info_.joints[i].parameters.end() ||
            info_.joints[i].parameters.find("rpi_id") == info_.joints[i].parameters.end())
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("HumanoidHardwareInterface"),
                "URDF joint '%s' is missing 'motor_id' or 'rpi_id' parameter.",
                joint_map_[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        try
        {
            joint_map_[i].motor_id = std::stoul(info_.joints[i].parameters.at("motor_id"));
            joint_map_[i].rpi_id = std::stoi(info_.joints[i].parameters.at("rpi_id"));
            joint_map_[i].direction = std::stoi(info_.joints[i].parameters.at("direction"));
        }
        catch (const std::invalid_argument & e)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("HumanoidHardwareInterface"),
                "Invalid 'motor_id' or 'rpi_id' for joint '%s'.", joint_map_[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint_map_[i].rpi_id == 1)
            motor_ids_rpi1_.push_back(joint_map_[i].motor_id);
        else if (joint_map_[i].rpi_id == 2)
            motor_ids_rpi2_.push_back(joint_map_[i].motor_id);
        else
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("HumanoidHardwareInterface"), "Invalid 'rpi_id' (%d) for joint '%s'. Must be 1 or 2.",
                joint_map_[i].rpi_id, joint_map_[i].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    auto find_index = [&](const std::string& name)->int {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        if (info_.joints[i].name == name) return static_cast<int>(i);
    }
    return -1;
    };

    idx_L_ankle_roll  = find_index("left_ankle_roll");
    idx_L_ankle_pitch = find_index("left_ankle_pitch");
    idx_R_ankle_roll  = find_index("right_ankle_roll");
    idx_R_ankle_pitch = find_index("right_ankle_pitch");
    
    // for (const auto & sensor : info_.sensors) 
    // {
    //     if (sensor.name == "imu_sensor") 
    //     {
    //         imu_sensor_name_ = sensor.name;
    //         has_imu_ = true;
    //         break;
    //     }
    // }

    // if (has_imu_) 
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"),
    //             "IMU sensor '%s' detected.", imu_sensor_name_.c_str());
    // }


    RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"), "Initialization successful.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"), "Configuring...");

    for (double &pos : hw_positions_) { pos = 0.0; }
    for (double &vel : hw_velocities_) { vel = 0.0; }
    for (double &cmd : hw_commands_) { cmd = 0.0; }

    try
    {
        udp_rpi1_comm_ = std::make_unique<UDPCommunication>("192.168.10.20", 5002, 6002);
        udp_rpi2_comm_ = std::make_unique<UDPCommunication>("192.168.10.30", 5003, 6003);
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("HumanoidHardwareInterface"),
            "Failed to create UDP communication objects: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"), "Configuration successful.");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn HumanoidHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"), "Activating...");

    if (!udp_rpi1_comm_ || !udp_rpi1_comm_->checkStart(motor_ids_rpi1_.data(), motor_ids_rpi1_.size()))
    {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidHardwareInterface"), "Failed to start RPI1");
        return hardware_interface::CallbackReturn::FAILURE;
    }
    if (!udp_rpi2_comm_ || !udp_rpi2_comm_->checkStart(motor_ids_rpi2_.data(), motor_ids_rpi2_.size()))
    {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidHardwareInterface"), "Failed to start RPI2");
        return hardware_interface::CallbackReturn::FAILURE;
    }

    read(rclcpp::Time(0), rclcpp::Duration(0, 0));

    for(size_t i = 0; i < hw_commands_.size(); ++i)
    {
        hw_commands_[i] = hw_positions_[i];
    }

    struct timespec ts;
    ts.tv_sec = 2;
    ts.tv_nsec = 0;
    nanosleep(&ts, nullptr);

    RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"), "Activation successful.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"), "Deactivating...");
    // UDP 소켓 등 리소스 정리
    udp_rpi1_comm_.reset();
    udp_rpi2_comm_.reset();
    RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"), "Deactivation successful.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HumanoidHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    // if (has_imu_) 
    // {
    //     state_interfaces.emplace_back(imu_sensor_name_, "orientation.x", &imu_.qx);
    //     state_interfaces.emplace_back(imu_sensor_name_, "orientation.y", &imu_.qy);
    //     state_interfaces.emplace_back(imu_sensor_name_, "orientation.z", &imu_.qz);
    //     state_interfaces.emplace_back(imu_sensor_name_, "orientation.w", &imu_.qw);

    //     state_interfaces.emplace_back(imu_sensor_name_, "angular_velocity.x", &imu_.wx);
    //     state_interfaces.emplace_back(imu_sensor_name_, "angular_velocity.y", &imu_.wy);
    //     state_interfaces.emplace_back(imu_sensor_name_, "angular_velocity.z", &imu_.wz);

    //     state_interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.x", &imu_.ax);
    //     state_interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.y", &imu_.ay);
    //     state_interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.z", &imu_.az);
    // }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HumanoidHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::return_type HumanoidHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::vector<MotorState> states_rpi1(motor_ids_rpi1_.size());
    std::vector<MotorState> states_rpi2(motor_ids_rpi2_.size());

    if (udp_rpi1_comm_) udp_rpi1_comm_->readState(states_rpi1.data(), states_rpi1.size());
    if (udp_rpi2_comm_) udp_rpi2_comm_->readState(states_rpi2.data(), states_rpi2.size());


    auto find_state = [&](uint16_t id, double& pos, double& vel)->bool {
        for (const auto& s: states_rpi1) if (s.drive_ids==id) { pos=s.position; vel=s.velocity; return true; }
        for (const auto& s: states_rpi2) if (s.drive_ids==id) { pos=s.position; vel=s.velocity; return true; }
        return false;
    };

    for (size_t i=0;i<joint_map_.size();++i)
    {
        if ((int)i==idx_L_ankle_roll || (int)i==idx_L_ankle_pitch ||
            (int)i==idx_R_ankle_roll || (int)i==idx_R_ankle_pitch) 
            continue;

        double pos, vel;
        if (find_state(joint_map_[i].motor_id, pos, vel))
        {
            hw_positions_[i]  = joint_map_[i].direction * pos;
            hw_velocities_[i] = joint_map_[i].direction * vel;
        }
    }

    if (idx_L_ankle_roll>=0 && idx_L_ankle_pitch>=0){
        const auto& jr = joint_map_[idx_L_ankle_roll];
        const auto& jp = joint_map_[idx_L_ankle_pitch];
        double p5, v5, p6, v6;
        if (find_state(jr.motor_id, p5, v5) && find_state(jp.motor_id, p6, v6))
        {
            const double t5  = jr.direction * p5;
            const double t6  = jp.direction * p6;
            const double t5d = jr.direction * v5;
            const double t6d = jp.direction * v6;

            auto rp  = ankle_kinematics::leftJointAngles(t5, t6);
            auto rpd = ankle_kinematics::footVelFromMotor(rp[0], rp[1], t5d, t6d, /*left=*/true);

            hw_positions_[idx_L_ankle_roll ] = rp[0];
            hw_positions_[idx_L_ankle_pitch] = rp[1];
            hw_velocities_[idx_L_ankle_roll ] = rpd[0];
            hw_velocities_[idx_L_ankle_pitch] = rpd[1];
        }
    }

    if (idx_R_ankle_roll>=0 && idx_R_ankle_pitch>=0){
        const auto& jr = joint_map_[idx_R_ankle_roll];
        const auto& jp = joint_map_[idx_R_ankle_pitch];
        double p5, v5, p6, v6;
        if (find_state(jr.motor_id, p5, v5) && find_state(jp.motor_id, p6, v6))
        {
            const double t5  = jr.direction * p5;
            const double t6  = jp.direction * p6;
            const double t5d = jr.direction * v5;
            const double t6d = jp.direction * v6;

            auto rp  = ankle_kinematics::rightJointAngles(t5, t6);
            auto rpd = ankle_kinematics::footVelFromMotor(rp[0], rp[1], t5d, t6d, /*left=*/false);

            hw_positions_[idx_R_ankle_roll ] = rp[0];
            hw_positions_[idx_R_ankle_pitch] = rp[1];
            hw_velocities_[idx_R_ankle_roll ] = rpd[0];
            hw_velocities_[idx_R_ankle_pitch] = rpd[1];
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HumanoidHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
    std::vector<MotorCommand> commands_rpi1;
    std::vector<MotorCommand> commands_rpi2;

    auto push_motor = [&](uint16_t motor_id, int rpi_id, double target)
    {
        MotorCommand cmd;
        cmd.drive_ids = motor_id;
        cmd.target_position = target;
        if (rpi_id == 1) 
            commands_rpi1.push_back(cmd);
        else             
            commands_rpi2.push_back(cmd);
    };

    auto push_with_dir = [&](const JointInfo& jinfo, double target_in_joint_space)
    {
        push_motor(jinfo.motor_id, jinfo.rpi_id, jinfo.direction * target_in_joint_space);
    };

    for (size_t i = 0; i < joint_map_.size(); ++i)
    {
        if ((int)i == idx_L_ankle_roll  || (int)i == idx_L_ankle_pitch ||
            (int)i == idx_R_ankle_roll  || (int)i == idx_R_ankle_pitch)
        {
            continue;
        }

        push_with_dir(joint_map_[i], hw_commands_[i]);
    }

    if (idx_L_ankle_roll >=0 && idx_L_ankle_pitch >=0)
    {
        const double r_cmd = hw_commands_[idx_L_ankle_roll ];
        const double p_cmd = hw_commands_[idx_L_ankle_pitch];

        ankle_kinematics::vec2 m = ankle_kinematics::leftMotorAngles(r_cmd, p_cmd);

        const JointInfo& m5 = joint_map_[idx_L_ankle_roll ];
        const JointInfo& m6 = joint_map_[idx_L_ankle_pitch];

        push_with_dir(m5, m[0]);
        push_with_dir(m6, m[1]);
    }

    if (idx_R_ankle_roll >=0 && idx_R_ankle_pitch >=0)
    {
        const double r_cmd = hw_commands_[idx_R_ankle_roll ];
        const double p_cmd = hw_commands_[idx_R_ankle_pitch];

        ankle_kinematics::vec2 m = ankle_kinematics::rightMotorAngles(r_cmd, p_cmd);

        const JointInfo& m5 = joint_map_[idx_R_ankle_roll ]; 
        const JointInfo& m6 = joint_map_[idx_R_ankle_pitch];

        push_with_dir(m5, m[0]);
        push_with_dir(m6, m[1]);
    }

    if (udp_rpi1_comm_ && !commands_rpi1.empty())
        udp_rpi1_comm_->sendCommand(commands_rpi1.data(), commands_rpi1.size());
    if (udp_rpi2_comm_ && !commands_rpi2.empty())
        udp_rpi2_comm_->sendCommand(commands_rpi2.data(), commands_rpi2.size());

    return hardware_interface::return_type::OK;
}

}  // namespace humanoid_hardware

PLUGINLIB_EXPORT_CLASS(
  humanoid_hardware::HumanoidHardwareInterface, 
  hardware_interface::SystemInterface
)