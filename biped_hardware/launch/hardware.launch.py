"""Real robot through the NUCLEO-H755 bridge: ros2_control + joint_state_broadcaster (+ RViz).

Step 2 (read path): BipedSystem never enables the motors.

Usage:
  ros2 launch biped_hardware hardware.launch.py
  ros2 launch biped_hardware hardware.launch.py rviz:=false board_ip:=192.168.10.10

Check:
  ros2 control list_hardware_interfaces
  ros2 topic hz /joint_states            # should be ~500 Hz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('biped_hardware')

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([pkg_share, 'urdf', 'biped_hw.urdf.xacro']),
            ' model:=', LaunchConfiguration('model'),
            ' board_ip:=', LaunchConfiguration('board_ip'),
        ]),
        value_type=str,
    )
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='v2_biped_robot_12dof.urdf',
            description='URDF file name inside biped_description/urdf'),
        DeclareLaunchArgument(
            'board_ip',
            default_value='192.168.10.10',
            description='NUCLEO-H755 address'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the biped_description config'),
        DeclareLaunchArgument(
            'base_height',
            default_value='0.86',
            description='Height of link0_torso above the world frame [m] (RViz only)'),

        # robot_description topic + /tf from /joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]),

        # Loads BipedSystem from the <ros2_control> block and runs read/update/write at update_rate
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description}, controllers_yaml]),

        # Publishes /joint_states from the position/velocity/effort state interfaces
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'),

        # world -> link0_torso until a state estimator publishes it
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_torso',
            output='screen',
            arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('base_height'),
                       '--frame-id', 'world', '--child-frame-id', 'link0_torso']),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('biped_description'), 'rviz', 'display.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
