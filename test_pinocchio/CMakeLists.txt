cmake_minimum_required(VERSION 3.16)
project(test_pinocchio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)

# ── Forward kinematics node ───────────────────────────────────────────────────
add_executable(forward_kinematics_node
  src/forward_kinematics_node.cpp
)

target_include_directories(forward_kinematics_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(forward_kinematics_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ocs2_centroidal_model
  ocs2_core
  ocs2_legged_robot
  ocs2_pinocchio_interface
)

install(TARGETS forward_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
