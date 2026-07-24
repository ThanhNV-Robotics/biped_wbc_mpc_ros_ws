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
# robot_state_visualization: find headers + library directly (it doesn't export a clean CMake target)
find_path(RSV_INCLUDE_DIR robot_state_visualization/mujoco_rendering.hpp
  HINTS ${CMAKE_INSTALL_PREFIX}/../robot_state_visualization/include
        ${AMENT_PREFIX_PATH}/include)
find_library(RSV_LIB robot_state_visualization_lib
  HINTS ${CMAKE_INSTALL_PREFIX}/../robot_state_visualization/lib
        ${AMENT_PREFIX_PATH}/lib)

# MuJoCo + GLFW (same logic as robot_state_visualization)
find_package(mujoco QUIET)
if(mujoco_FOUND)
  set(MUJOCO_LIB mujoco::mujoco)
elseif(DEFINED ENV{MUJOCO_DIR})
  find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib)
  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
else()
  message(FATAL_ERROR "MuJoCo not found. Set MUJOCO_DIR or install from source.")
endif()
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h)
find_library(MUJOCO_LIB mujoco)

# ── Forward kinematics node (with MuJoCo visualizer) ─────────────────────────
add_executable(forward_kinematics_node
  src/forward_kinematics_node.cpp
)

target_include_directories(forward_kinematics_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${MUJOCO_INCLUDE_DIR}
  ${GLFW_INCLUDE_DIRS}
  ${RSV_INCLUDE_DIR}
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

target_link_libraries(forward_kinematics_node
  ${MUJOCO_LIB}
  ${GLFW_LIBRARIES}
  ${RSV_LIB}
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
