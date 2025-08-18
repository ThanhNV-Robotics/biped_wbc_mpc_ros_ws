#include "humanoid_hardware/humanoid_hardware_interface.hpp"
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
    hw_commands_.resize(num_joints, std::numeric_limits<double>::quiet_NaN());
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
        }
        catch (const std::invalid_argument & e)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("HumanoidHardwareInterface"),
            "Invalid 'motor_id' or 'rpi_id' for joint '%s'.", joint_map_[i].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint_map_[i].rpi_id == 1)
        {
        motor_ids_rpi1_.push_back(joint_map_[i].motor_id);
        }
        else if (joint_map_[i].rpi_id == 2)
        {
        motor_ids_rpi2_.push_back(joint_map_[i].motor_id);
        }
        else
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("HumanoidHardwareInterface"), "Invalid 'rpi_id' (%d) for joint '%s'. Must be 1 or 2.",
            joint_map_[i].rpi_id, joint_map_[i].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"), "Initialization successful.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HumanoidHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("HumanoidHardwareInterface"), "Activating...");

    udp_rpi1_comm_ = std::make_unique<UDPCommunication>("192.168.10.20", 5002, 6002);
    udp_rpi2_comm_ = std::make_unique<UDPCommunication>("192.168.10.30", 5003, 6003);

    if (!udp_rpi1_comm_->checkStart(motor_ids_rpi1_.data(), motor_ids_rpi1_.size()))
    {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidHardwareInterface"), "Failed to start RPI1");
        return hardware_interface::CallbackReturn::FAILURE;
    }
    if (!udp_rpi2_comm_->checkStart(motor_ids_rpi2_.data(), motor_ids_rpi2_.size()))
    {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidHardwareInterface"), "Failed to start RPI2");
        return hardware_interface::CallbackReturn::FAILURE;
    }

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
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_torques_[i]));
    }
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
    MotorState states_rpi1[motor_ids_rpi1_.size()];
    if (udp_rpi1_comm_ && udp_rpi1_comm_->readState(states_rpi1, motor_ids_rpi1_.size()))
    {
        for (size_t i = 0; i < motor_ids_rpi1_.size(); ++i)
        {
        auto it = std::find_if(joint_map_.begin(), joint_map_.end(), 
            [&](const JointInfo& j){ return j.motor_id == states_rpi1[i].drive_ids; });
        if (it != joint_map_.end()) {
            size_t joint_idx = std::distance(joint_map_.begin(), it);
            hw_positions_[joint_idx] = states_rpi1[i].position;
            hw_velocities_[joint_idx] = states_rpi1[i].velocity;
            hw_torques_[joint_idx] = states_rpi1[i].torque;
        }
        }
    }

    MotorState states_rpi2[motor_ids_rpi2_.size()];
    if (udp_rpi2_comm_ && udp_rpi2_comm_->readState(states_rpi2, motor_ids_rpi2_.size()))
    {
        for (size_t i = 0; i < motor_ids_rpi2_.size(); ++i)
        {
        auto it = std::find_if(joint_map_.begin(), joint_map_.end(), 
            [&](const JointInfo& j){ return j.motor_id == states_rpi2[i].drive_ids; });
        if (it != joint_map_.end()) {
            size_t joint_idx = std::distance(joint_map_.begin(), it);
            hw_positions_[joint_idx] = states_rpi2[i].position;
            hw_velocities_[joint_idx] = states_rpi2[i].velocity;
            hw_torques_[joint_idx] = states_rpi2[i].torque;
        }
        }
    }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HumanoidHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::vector<MotorCommand> commands_rpi1;
    std::vector<MotorCommand> commands_rpi2;

    for (size_t i = 0; i < joint_map_.size(); ++i)
    {
        MotorCommand cmd;
        cmd.drive_ids = joint_map_[i].motor_id;
        cmd.target_position = hw_commands_[i]; 
        // cmd.kp = 5.0;
        // cmd.kd = 0.1;
        // cmd.target_velocity = 0.0;
        // cmd.target_torque = 0.0;

        if (joint_map_[i].rpi_id == 1)
        {
            commands_rpi1.push_back(cmd);
        }
        else
        {
            commands_rpi2.push_back(cmd);
        }
    }

    if (udp_rpi1_comm_ && !commands_rpi1.empty())
    {
        udp_rpi1_comm_->sendCommand(commands_rpi1.data(), commands_rpi1.size());
    }
    if (udp_rpi2_comm_ && !commands_rpi2.empty())
    {
        udp_rpi2_comm_->sendCommand(commands_rpi2.data(), commands_rpi2.size());
    }

  return hardware_interface::return_type::OK;
}

}  // namespace humanoid_hardware

PLUGINLIB_EXPORT_CLASS(
  humanoid_hardware::HumanoidHardwareInterface, 
  hardware_interface::SystemInterface
)