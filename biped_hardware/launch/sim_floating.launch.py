"""MuJoCo floating-base simulation of the v2 robot, standing on the floor and held at the home pose
by position_controller. Same ros2_control setup as sim.launch.py (mujoco_ros2_control PVT mode,
config/controllers.yaml, config/joint_gains_sim.yaml); differences:

  - Scene scene_floatingbase_12dof_v2.xml, started from its keyframe `home` (feet on the floor).
  - BipedSystem is ACTIVE from startup (not inactive as on the robot): the physics runs from the
    start, and limp motors would let the robot collapse before an activation call. Activation
    holds the keyframe pose.
  - position_controller (ForwardCommandController, position only; velocity / effort commands stay
    0) is spawned and sent HOME once. It keeps sending the last command every cycle, so the drive
    never times out.
  - TF world -> link0_torso from the simulator's floating-base odometry (scripts/odom_to_tf.py)
    instead of a static transform.

Usage:
  ros2 launch biped_hardware sim_floating.launch.py
  ros2 launch biped_hardware sim_floating.launch.py headless:=true rviz:=false

Other poses, in the joint order of position_controller in controllers.yaml:
  ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [...]}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Home pose per leg: hip pitch, hip roll, hip yaw, knee, ankle roll, ankle pitch [rad].
# Same values as the `home` keyframe in biped_description/mjcf/scene_floatingbase_12dof_v2.xml.
HOME_LEG = [-0.2, 0.0, 0.0, 0.4, 0.0, -0.2]
HOME = HOME_LEG + HOME_LEG  # left, right


def generate_launch_description():
    pkg_share = FindPackageShare('biped_hardware')
    use_sim_time = {'use_sim_time': True}   # /clock from the MuJoCo physics thread

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([pkg_share, 'urdf', 'biped_hw.urdf.xacro']),
            ' use_sim:=true',
            ' model:=', LaunchConfiguration('model'),
            ' gains:=', LaunchConfiguration('gains'),
            ' mujoco_scene:=', LaunchConfiguration('mujoco_scene'),
            ' initial_keyframe:=', LaunchConfiguration('initial_keyframe'),
            ' headless:=', LaunchConfiguration('headless'),
        ]),
        value_type=str,
    )
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '--controller-manager', '/controller_manager'],
        output='screen')
    start_position_controller = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[position_spawner]))

    # Until the first message the controller writes nothing, so the drive keeps the activation
    # hold (= the keyframe); from here on it holds HOME explicitly
    send_home = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '-w', '1', '/position_controller/commands',
             'std_msgs/msg/Float64MultiArray', '{data: [' + ', '.join(map(str, HOME)) + ']}'],
        output='screen')
    start_send_home = RegisterEventHandler(
        OnProcessExit(target_action=position_spawner, on_exit=[send_home]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='v2_biped_robot_12dof.urdf',
            description='URDF file name inside biped_description/urdf'),
        DeclareLaunchArgument(
            'gains',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'joint_gains_sim.yaml']),
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
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the biped_description config'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}, use_sim_time]),

        # controller_manager with the MuJoCo hardware; its loop runs on the simulation clock.
        # controllers_sim_floating.yaml after controllers.yaml: BipedSystem active at startup
        Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description}, controllers_yaml,
                        PathJoinSubstitution([pkg_share, 'config', 'controllers_sim_floating.yaml']),
                        use_sim_time]),

        jsb_spawner,
        start_position_controller,
        start_send_home,

        Node(
            package='biped_hardware',
            executable='odom_to_tf.py',
            output='screen',
            parameters=[use_sim_time]),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            parameters=[use_sim_time],
            arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('biped_description'), 'rviz', 'display.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
