from ament_index_python.packages import get_package_share_directory

import launch
from humanoid_common_mpc_ros2.mpc_launch_config import MPCLaunchConfig
from launch_ros.actions import Node

# For mujoco_ros2_control

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_mujoco_ros2_control_setup(context, *args, **kwargs):
    biped_pkg_share = FindPackageShare("biped_description")
    controller_pkg_share = FindPackageShare("biped_joint_pd_impedance_controller")

    # Build robot description for biped robot
    # Note: use_pid is set to true to export velocity/effort command interfaces
    # The MuJoCo interface will use custom PD impedance mode when all 3 interfaces are claimed
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([biped_pkg_share, "urdf", "biped_robot_12dof.urdf"]),
            " use_pid:=true",  # Export position, velocity, effort interfaces
            " headless:=",
            LaunchConfiguration("headless"),
            " use_mjcf_from_topic:=false",
            " use_transmissions:=false",
        ]
    )

    robot_description_str = robot_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    # Controller configuration file
    parameters_file = PathJoinSubstitution([controller_pkg_share, "config", "pd_impedance_controller.yaml"])

    nodes = []

    # Robot state publisher
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    # ros2_control node with MuJoCo - PD gains loaded from config
    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(parameters_file),
            ],
            # remappings removed to avoid duplicate URDF loading
            on_exit=Shutdown(),
        )
    )

    # Controller spawners
    # Note: We spawn the PD impedance controller
    controllers_to_spawn = ["joint_state_broadcaster", "pd_impedance_controller", "imu_sensor_broadcaster"]
    for controller in controllers_to_spawn:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", parameters_file],
                output="both",
            )
        )

    return nodes

def generate_launch_description():

    # mujoco_ros2_control setup
    headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run simulation without visualization window",
    )

    mujoco_ros2_control_launch_description  = LaunchDescription(
        [
            headless,
            OpaqueFunction(function=launch_mujoco_ros2_control_setup),
        ]
    )

    cfg = MPCLaunchConfig(
            mpc_lib_pkg="humanoid_centroidal_mpc",
            mpc_config_pkg="biped_centroidal_mpc",
            mpc_model_pkg="biped_description",
            urdf_rel_path="/urdf/biped_robot_12dof.urdf",
            xml_rel_path="/mjcf/biped_robot_12dof.xml",
            robot_name="biped_robot",
            solver="sqp",
            enable_debug=False,
        )

    # Add parameters
    cfg.ld.add_action(cfg.declare_robot_name)
    cfg.ld.add_action(cfg.declare_config_file)
    cfg.ld.add_action(cfg.declare_target_command_file)
    cfg.ld.add_action(cfg.declare_gait_command_file)
    cfg.ld.add_action(cfg.declare_urdf_path)
    cfg.ld.add_action(cfg.declare_rviz_config_path)
    cfg.ld.add_action(cfg.declare_xml_path)

    # Add nodes
    # cfg.ld.add_action(cfg.mpc_sim)
    # cfg.ld.add_action(cfg.robot_state_publisher_node)
    # cfg.ld.add_action(cfg.terminal_robot_state_publisher_node) # can be ignored since we already have robot_state_publisher_node in the mujoco_ros2_control launch
    # cfg.ld.add_action(cfg.target_robot_state_publisher_node)
    # cfg.ld.add_action(cfg.rviz_node)
    cfg.ld.add_action(cfg.base_velocity_controller_gui_node)

    final_launch_description = LaunchDescription()
    
    final_launch_description.add_action(mujoco_ros2_control_launch_description)
    final_launch_description.add_action(cfg.ld)

    return final_launch_description
