import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    mujoco_ros2_control_demos_path = os.path.join(
        get_package_share_directory('mujoco_ros2_control_demos'))

    xacro_file = os.path.join(mujoco_ros2_control_demos_path,
                              'urdf',
                              'left_leg_position_pd_cmd.urdf') # read urdf file

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}# convert to xml
    # define the controller yaml config file
    controller_config_file = os.path.join(mujoco_ros2_control_demos_path, 'config', 'left_leg_position_trajectory_pd_controller.yaml') 

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'Left_Leg.xml')}
        ]
    ) # mujoco simulator node

    node_mujoco_ros2_mimic = Node(
        package='robot_state_visualization',
        executable='robot_state_visualization_node',
        output='screen',
        parameters=[
            {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'Left_Leg.xml')}
        ]
    ) # mujoco mimic node

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    ) # robot_state_publisher node

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    ) # publish joint states (position and velocity), load this controller and set to active

    load_joint_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    ) # at the beginning, load the joint_trajectory_controller and set to active to claim the state interface 

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_position_controller],
            )
        ),
        node_mujoco_ros2_control,
        node_mujoco_ros2_mimic,
        node_robot_state_publisher
    ])
