from ament_index_python.packages import get_package_share_directory

import launch
from humanoid_common_mpc_ros2.mpc_launch_config import MPCLaunchConfig
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
import os
import xacro


def generate_launch_description():

    cfg = MPCLaunchConfig(
        mpc_lib_pkg="humanoid_centroidal_mpc",
        mpc_config_pkg="gist_biped_centroidal_mpc",
        mpc_model_pkg="biped_description",
        urdf_rel_path="/urdf/biped_12dof.urdf",
        xml_rel_path="/mjcf/biped_robot_12dof.xml",
        robot_name="biped_12dof_robot",
        solver="sqp",
        enable_debug=False,
    )
    
    # Override rviz config path to use biped_description rviz config
    biped_rviz_config = os.path.join(
        get_package_share_directory("biped_description"),
        "rviz/urdf_config.rviz"
    )
    cfg.declare_rviz_config_path = DeclareLaunchArgument(
        "rvizconfig",
        default_value=biped_rviz_config,
        description="Absolute path to rviz config file",
    )

    # Add parameters
    cfg.ld.add_action(cfg.declare_robot_name)
    cfg.ld.add_action(cfg.declare_config_file)
    cfg.ld.add_action(cfg.declare_target_command_file)
    cfg.ld.add_action(cfg.declare_gait_command_file)
    cfg.ld.add_action(cfg.declare_urdf_path)
    cfg.ld.add_action(cfg.declare_rviz_config_path)
    cfg.ld.add_action(cfg.declare_xml_path)

# #--------------------------------------------
#     # For mujoco_ros2_control
#     # Get gist_biped_centroidal_mpc package path for controller config
#     gist_biped_pkg_path = get_package_share_directory("gist_biped_centroidal_mpc")
    
#     # Define the controller yaml config file from gist_biped_centroidal_mpc package
#     controller_config_file = os.path.join(gist_biped_pkg_path, 'config', 'joint_controller', 'bipedal_robot_position_trajectory_controller.yaml')
    

#     doc = xacro.parse(open(cfg.urdf_path))
#     xacro.process_doc(doc)
#     robot_description = {'robot_description': doc.toxml()}# convert to xml
#     # For mujoco_ros2_control node
#     mujoco_ros2_control_node = Node(
#         package='mujoco_ros2_control',
#         executable='mujoco_ros2_control',
#         output='screen',
#         parameters=[
#             robot_description,
#             controller_config_file,
#             {'mujoco_model_path': cfg.xml_path,
#              'use_robot_description_topic': False}
#         ]
#     )

#     # to publish robot_description
#     # to publish robot_description
#     node_robot_state_publisher = Node( 
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description]
#     ) # robot_state_publisher node

#     load_joint_state_broadcaster = ExecuteProcess(
#         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#              'joint_state_broadcaster'],
#         output='screen'
#     ) # publish joint states (position and velocity), load this controller and set to active

#     # Add nodes with proper timing
#     cfg.ld.add_action(node_robot_state_publisher)
#     # Start mujoco_ros2_control after a short delay to ensure robot_description is available
#     cfg.ld.add_action(TimerAction(
#         period=2.0,
#         actions=[mujoco_ros2_control_node]
#     ))
#     # Load joint_state_broadcaster when mujoco_ros2_control starts
#     cfg.ld.add_action(RegisterEventHandler(
#         event_handler=OnProcessStart(
#             target_action=mujoco_ros2_control_node,
#             on_start=[TimerAction(period=2.0, actions=[load_joint_state_broadcaster])]
#         )
#     ))
    # cfg.ld.add_action(cfg.mpc_sim)
    cfg.ld.add_action(cfg.robot_state_publisher_node)
    cfg.ld.add_action(cfg.terminal_robot_state_publisher_node)
    cfg.ld.add_action(cfg.target_robot_state_publisher_node)
    cfg.ld.add_action(cfg.rviz_node)
    cfg.ld.add_action(cfg.base_velocity_controller_gui_node)

    return cfg.ld
