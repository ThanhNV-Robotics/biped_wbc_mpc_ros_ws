import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory('humanoid_hardware')
    # mujoco_ros2_control_demos_path = os.path.join(
    #     get_package_share_directory('mujoco_ros2_control_demos'))
    
    xacro_file = os.path.join(pkg_path, 'urdf', 'humanoid.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    controller_config = os.path.join(pkg_path, 'config', 'humanoid_controllers.yaml')

    # node_mujoco_ros2_mimic = Node(
    #     package='robot_state_visualization',
    #     executable='robot_state_visualization_node',
    #     output='screen',
    #     parameters=[
    #         {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'Bipedal_Robot_Fixed_Base.xml')}
    #     ]
    # ) # mujoco mimic node

    # node_control_action = Node(
    #         package='control_actions',
    #         executable='control_actions_server_node',
    #         output='screen',
    #         parameters=[
    #             {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'mujoco_models', 'Bipedal_Robot_Fixed_Base.xml')}
    #         ]
    #     ) # control actions server node


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        # node_mujoco_ros2_mimic,
        # node_control_action,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        imu_sensor_broadcaster_spawner
    ])