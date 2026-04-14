import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Setup function to access launch configurations"""
    
    # Package directories
    biped_description_share = FindPackageShare("biped_description")
    biped_centroidal_mpc_share = FindPackageShare("biped_centroidal_mpc")
    
    # Build robot description with xacro for biped robot
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([biped_description_share, "urdf", "biped_robot_12dof.urdf"]),
            " use_pid:=true",  # Enable PID control for mujoco_ros2_control
            " headless:=",
            LaunchConfiguration("headless"),
            " use_mjcf_from_topic:=false",
            " use_transmissions:=false",
        ]
    )
    
    robot_description_str = robot_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}
    
    # Controller configuration file
    joint_controller_config_file = PathJoinSubstitution([
        biped_description_share, 
        "config", 
        "bipedal_robot_position_trajectory_controller.yaml"
    ])
    
    # MPC configuration files
    mpc_config_file = PathJoinSubstitution([
        biped_centroidal_mpc_share,
        "config",
        "mpc",
        "task.info"
    ])
    
    target_command_file = PathJoinSubstitution([
        biped_centroidal_mpc_share,
        "config",
        "target_command.info"
    ])
    
    gait_file = PathJoinSubstitution([
        biped_centroidal_mpc_share,
        "config",
        "gait.info"
    ])

    ########### Define nodes ###########
    
    nodes = []
    
    # 1. Robot State Publisher
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
            namespace="",
        )
    )
    
    # 2. MuJoCo ros2_control node - This simulates the robot hardware
    mujoco_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        emulate_tty=True,
        output="both",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(joint_controller_config_file),
        ],
        remappings=(
            [("~/robot_description", "/robot_description")] if os.environ.get("ROS_DISTRO") == "humble" else []
        ),
    )
    nodes.append(mujoco_control_node)
    
    # 3. Joint State Broadcaster - Publishes /joint_states topic
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", joint_controller_config_file],
        output="both",
    )
    
    # 4. IMU Sensor Broadcaster - Publishes /imu_sensor_broadcaster/imu topic
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--param-file", joint_controller_config_file],
        output="both",
    )
    
    # # 5. MPC SQP Node - The main MPC controller
    # mpc_node = Node(
    #     package="humanoid_centroidal_mpc_ros2",
    #     executable="humanoid_centroidal_mpc_sqp_node",
    #     output="both",
    #     arguments=[
    #         "biped_robot",  # robot_name
    #         str(mpc_config_file.perform(context)),  # config_file
    #         str(target_command_file.perform(context)),  # target_command_file
    #         PathJoinSubstitution([biped_description_share, "urdf", "biped_robot_12dof.urdf"]).perform(context),  # urdf_path
    #         str(gait_file.perform(context)),  # gait_file
    #     ],
    #     parameters=[{"use_sim_time": True}],
    # )
    
    # 5. Controller spawners (spawn immediately after setup)
    nodes.append(joint_state_broadcaster_spawner)
    nodes.append(imu_sensor_broadcaster_spawner)
    
    # 6. Velocity Command GUI
    velocity_gui_node = Node(
        package="remote_control",
        executable="base_velocity_controller_gui",
        name="base_velocity_controller_gui",
        output="both",
        parameters=[{"use_sim_time": True}],
    )
    nodes.append(velocity_gui_node)
    
    # 7. MPC Node (commented out for now)
    # nodes.append(mpc_node)
    
    return nodes


def generate_launch_description():
    """Generate launch description with launch arguments"""
    
    # Declare launch arguments
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run MuJoCo simulation without visualization window",
    )
    
    return LaunchDescription(
        [
            headless_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
