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
HOME_LEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
HOME = HOME_LEG + HOME_LEG  # left, right legs

def generate_launch_description():
    # get the launch directory
    pkg_share = FindPackageShare('biped_hardware')
    use_sim_time = {'use_sim_time': True}   # /clock from the MuJoCo physics thread

    # 1. Build the robot description when the launch file loads
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

    position_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '--controller-manager', '/controller_manager'],
        output='screen')
    
    
    return LaunchDescription([
        # Those arguments are passed to xacro file
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

        # Core node from mujoco_ros2_control
        # it:
        #  - run mujoco simulator, 
        #  - run controller manager, 
        #  - run the mujoco system life cycle (hardware plugin)
        #  - run controllers' loop
        #  - offer controller manager's services

        Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            output='screen',
            parameters=[{'robot_description': robot_description}, controllers_yaml,
                        PathJoinSubstitution([pkg_share, 'config', 'controllers_sim_floating.yaml']),
                        use_sim_time]),
                        
        position_controllers,     
    ])