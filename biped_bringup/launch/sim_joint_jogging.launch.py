"""MuJoCo simulation of the robot: same ros2_control setup as hardware.launch.py, BipedSystem
replaced by mujoco_ros2_control in PVT mode (urdf/biped_hw.urdf.xacro use_sim:=true).

The simulated hardware behaves like the robot: same component name (BipedSystem), joints,
interfaces (position / velocity / effort commands and states, BipedSystem/enabled) and gain law
(fixed kp / kd per joint, tau = kp (p - q) + kd (v - dq) + tau_ff). It starts INACTIVE (motors
disabled, limp); activating it checks the joint ranges and holds the current pose; deactivating
makes it limp again; no command for 20 ms (sim time) -> damping 1 s -> disabled, and read()
reports the error as BipedSystem does. The sim ankle is a serial roll / pitch chain.

Default scene: the robot hangs 1.0 m above the floor (torso welded to the world), for checking
the framework. The controllers come from the same config/controllers.yaml; the gains from
section `robot` of config/joint_gains.yaml, as the robot (ankles: sum of the two motors' gains).

Usage:
  ros2 launch biped_bringup sim.launch.py                    # motors stay disabled (limp)
  ros2 launch biped_bringup sim.launch.py enable:=true       # enabled, holding the pose
  ros2 launch biped_bringup sim.launch.py slider:=true       # enabled + sliders (jogging)
  ros2 launch biped_bringup sim.launch.py headless:=true rviz:=false
  ros2 launch biped_bringup sim.launch.py gains:=/path/to/my_gains.yaml

By hand, as on the robot:
  ros2 control set_hardware_component_state BipedSystem active     # enable
  ros2 control set_hardware_component_state BipedSystem inactive   # disable
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
    use_sim_time = {'use_sim_time': True}   # /clock from the MuJoCo physics thread

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

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    enable = LaunchConfiguration('enable')
    slider = LaunchConfiguration('slider')

    # Same chain as hardware.launch.py: activate once the spawner is done; a refused activation
    # leaves the hardware inactive and the log says why
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

    test_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_jogging_controller', '--controller-manager', '/controller_manager'],
        output='screen')
    start_test_controllers = RegisterEventHandler(
        OnProcessExit(target_action=activate, on_exit=[test_controllers]),
        condition=IfCondition(slider))

    sliders = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[use_sim_time],
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
            'gains',
            default_value=PathJoinSubstitution([hw_share, 'config', 'joint_gains.yaml']),
            description='kp / kd per joint, fixed while active'),
        DeclareLaunchArgument(
            'mujoco_scene',
            default_value='scene_hanging_12dof_v2.xml',
            description='MJCF scene inside biped_description/mjcf'),
        DeclareLaunchArgument(
            'initial_keyframe',
            default_value='hang',
            description='Keyframe of the scene to start from'),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run MuJoCo without its viewer window'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the biped_description config'),
        DeclareLaunchArgument(
            'enable',
            default_value='false',
            description='Enable the simulated motors (activate BipedSystem) right after startup'),
        DeclareLaunchArgument(
            'slider',
            default_value='false',
            description='Enable the motors and move them with joint_state_publisher_gui sliders'),
        DeclareLaunchArgument(
            'base_height',
            default_value='1.0',
            description='Height of link0_torso above the world frame [m] (RViz only; hanging scene: 1.0)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}, use_sim_time]),

        # controller_manager with the MuJoCo hardware; its loop runs on the simulation clock
        Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description}, controllers_yaml, use_sim_time]),

        jsb_spawner,
        enable_hardware,
        start_test_controllers,
        start_sliders,

        # world -> link0_torso: fixed in the hanging scene
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
            parameters=[use_sim_time],
            arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('biped_description'), 'rviz', 'display.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
