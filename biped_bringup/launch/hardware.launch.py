"""Real robot through the NUCLEO-H755 bridge: ros2_control + joint_state_broadcaster (+ RViz).

BipedSystem starts INACTIVE (states only, motors disabled). With enable:=true the launch file
activates it as soon as joint_state_broadcaster is up (motors enabled, holding the current pose
with their own kp / kd: PVT commands only). If the activation is refused (motor offline, joint outside its URDF limits, ...),
the hardware stays inactive, ROS keeps running and the log says why.

With slider:=true (implies enable:=true) it also starts joint_jogging_controller (package
biped_controllers) and joint_state_publisher_gui (sliders, remapped to /slider_joint_states). The
controller moves each joint slowly and smoothly to its slider value once that slider has been moved
(speed / filter: joint_jogging_controller in config/controllers.yaml). For checking read / write
on the robot.

Usage:
  ros2 launch biped_bringup hardware.launch.py                 # motors stay disabled
  ros2 launch biped_bringup hardware.launch.py enable:=true    # motors enabled, holding the pose
  ros2 launch biped_bringup hardware.launch.py slider:=true    # enabled + sliders
  ros2 launch biped_bringup hardware.launch.py rviz:=false board_ip:=192.168.10.10
  ros2 launch biped_bringup hardware.launch.py slider:=true gains:=/path/to/my_gains.yaml

By hand:
  ros2 control set_hardware_component_state BipedSystem active     # enable
  ros2 control set_hardware_component_state BipedSystem inactive   # disable

Check:
  ros2 control list_hardware_interfaces
  ros2 topic hz /joint_states            # should be ~500 Hz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_share = FindPackageShare('biped_hardware')        # xacro, joint gains
    bringup_share = FindPackageShare('biped_bringup')    # controllers yaml, scripts

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([hw_share, 'urdf', 'biped_hw.urdf.xacro']),
            ' model:=', LaunchConfiguration('model'),
            ' board_ip:=', LaunchConfiguration('board_ip'),
            ' gains:=', LaunchConfiguration('gains'),
        ]),
        value_type=str,
    )
    controllers_yaml = PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml'])

    # Publishes /joint_states from the position/velocity/effort state interfaces
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    enable = LaunchConfiguration('enable')
    slider = LaunchConfiguration('slider')

    # enable:=true (or slider:=true): activate BipedSystem once the spawner is done
    # (controller_manager is up and the hardware is configured). Same as the command line call, so
    # a refused activation only leaves the hardware inactive instead of stopping ros2_control_node.
    activate = ExecuteProcess(
        # The service directly, not `ros2 control set_hardware_component_state`: that one goes
        # through the ros2 daemon, and a stale daemon fails the call with "!rclpy.ok()"
        cmd=['ros2', 'service', 'call', '/controller_manager/set_hardware_component_state',
             'controller_manager_msgs/srv/SetHardwareComponentState',
             "{name: BipedSystem, target_state: {id: 3, label: active}}"],
        output='screen')
    enable_hardware = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[activate]),
        condition=IfCondition(PythonExpression(
            ["'", enable, "' == 'true' or '", slider, "' == 'true'"])))

    # slider:=true, step 1: joint_jogging_controller. Its command interfaces only exist once the
    # hardware is active, so it starts after the activation.
    test_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_jogging_controller', '--controller-manager', '/controller_manager'],
        output='screen')
    start_test_controllers = RegisterEventHandler(
        OnProcessExit(target_action=activate, on_exit=[test_controllers]),
        condition=IfCondition(slider))

    # slider:=true, step 2: the sliders (the controller subscribes to them)
    sliders = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        # not /joint_states: that topic is the real robot (joint_state_broadcaster)
        remappings=[('joint_states', 'slider_joint_states')])
    start_sliders = RegisterEventHandler(
        OnProcessExit(target_action=test_controllers, on_exit=[sliders]),
        condition=IfCondition(slider))

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
            'gains',
            default_value=PathJoinSubstitution([hw_share, 'config', 'joint_gains.yaml']),
            description='kp / kd per joint, written to the motors at activation, then fixed'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the biped_description config'),
        DeclareLaunchArgument(
            'enable',
            default_value='false',
            description='Enable the motors (activate BipedSystem) right after startup'),
        DeclareLaunchArgument(
            'slider',
            default_value='false',
            description='Enable the motors and move them with joint_state_publisher_gui sliders'),
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

        jsb_spawner,
        enable_hardware,
        start_test_controllers,
        start_sliders,

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
