import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    mujoco_ros2_control_demos_path = os.path.join(
        get_package_share_directory('robot_state_visualization'))

    node_mujoco_ros2_mimic = Node(
        package='robot_state_visualization',
        executable='robot_state_visualization_node',
        output='screen',
        parameters=[
            {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'Bipedal_Robot_Mimic.xml')}
        ]
    ) # mujoco simulator node


    return LaunchDescription([
        node_mujoco_ros2_mimic
    ])
