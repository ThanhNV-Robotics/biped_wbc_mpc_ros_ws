"""MuJoCo floating-base simulation running the whole-body controller (biped_controllers/WBCController).

Same simulation as sim_floating.launch.py (mujoco_ros2_control in PVT mode, BipedSystem active
from startup, motors holding the keyframe pose), but instead of position_controller the WBC is
started. Until the WBC writes commands, the drive keeps the activation hold (keyframe pose).

Startup chain (each spawner starts when the previous one exits):
  joint_state_broadcaster -> imu_sensor_broadcaster -> wbc_controller

Usage:
  ros2 launch biped_bringup tsid_wbc.launch.py
  ros2 launch biped_bringup tsid_wbc.launch.py headless:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_share = FindPackageShare('biped_hardware')        # xacro, joint gains
    bringup_share = FindPackageShare('biped_bringup')    # controllers yaml, scripts
    use_sim_time = {'use_sim_time': True}   # /clock from the MuJoCo physics thread

    # 1. Build the robot description when the launch file loads (arguments -> xacro)
    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([hw_share, 'urdf', 'biped_hw.urdf.xacro']),
            ' use_sim:=true',
            ' model:=', LaunchConfiguration('model'),
            ' gains:=', LaunchConfiguration('gains'),
            ' mujoco_scene:=', LaunchConfiguration('mujoco_scene'),
            ' initial_keyframe:=', LaunchConfiguration('initial_keyframe'),
            ' headless:=', LaunchConfiguration('headless'),
        ]),
        value_type=str,
    )

    controllers_yaml = PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml'])
    # BipedSystem active at startup (floating base: limp motors would let the robot fall)
    sim_floating_yaml = PathJoinSubstitution([bringup_share, 'config', 'controllers_sim_floating.yaml'])

    # 2. Controllers, started one after the other
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    imu_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    wbc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wbc_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    start_imu = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[imu_spawner]))
    start_wbc = RegisterEventHandler(
        OnProcessExit(target_action=imu_spawner, on_exit=[wbc_spawner]))

    return LaunchDescription([
        # Arguments passed to the xacro file
        DeclareLaunchArgument(
            'model',
            default_value='v2_biped_robot_12dof.urdf',
            description='URDF file name inside biped_description/urdf'),
        DeclareLaunchArgument(
            'gains',
            default_value=PathJoinSubstitution([hw_share, 'config', 'joint_gains_sim.yaml']),
            description='kp / kd per joint, fixed while active'),
        DeclareLaunchArgument(
            'mujoco_scene',
            default_value='scene_floatingbase_12dof_v2.xml',
            description='MJCF scene inside biped_description/mjcf'),
        DeclareLaunchArgument(
            'initial_keyframe',
            default_value='home',
            description='Keyframe of the scene to start from'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run MuJoCo without its viewer window'),

        # Core node from mujoco_ros2_control. It:
        #  - runs the MuJoCo simulation (physics thread, viewer)
        #  - runs the controller manager and its update loop (500 Hz, sim time)
        #  - runs the hardware plugin lifecycle (BipedSystem = MujocoSystemInterface)
        #  - offers the controller manager services (load / switch / list ...)
        Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description}, controllers_yaml,
                        sim_floating_yaml, use_sim_time]),

        jsb_spawner,   # publishes /joint_states (monitoring, robot_state_publisher)
        start_imu,     # publishes /imu_sensor_broadcaster/imu (monitoring)
        start_wbc,     # whole-body controller

        # /tf of the links from /joint_states, /robot_description (not needed by the controllers)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}, use_sim_time]),
    ])
