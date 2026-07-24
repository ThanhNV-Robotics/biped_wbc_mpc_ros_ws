cmake_minimum_required(VERSION 3.5)
project(robot_state_visualization)

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ocs2_msgs REQUIRED)
# MuJoCo + GLFW
find_package(mujoco QUIET)
if(mujoco_FOUND)
  message(STATUS "Mujoco build from source has been found")
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
elseif(DEFINED ENV{MUJOCO_DIR})
  message(STATUS "Mujoco build from source has not been found. Attempting to find the binary in $ENV{MUJOCO_DIR} instead.")
  find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "Failed to find binary in $ENV{MUJOCO_DIR}")
  endif()
  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
else()
  message(FATAL_ERROR "Failed to find mujoco with find_package.
  Either build and install mujoco from source or set the MUJOCO_DIR environment variable to tell CMake where to find the binary install. ")
endif()
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h)
find_library(MUJOCO_LIB mujoco)


# Include directories
include_directories(
  include
  ${MUJOCO_INCLUDE_DIR}
  ${GLFW_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Library from rendering + cameras
add_library(robot_state_visualization_lib
  src/mujoco_rendering.cpp
  src/standing_balance_control.cpp
  src/computation_utils.cpp
)

# ament_target_dependencies(robot_state_visualization_lib
#   rclcpp
#   sensor_msgs
#   control_actions
# )

target_link_libraries(robot_state_visualization_lib
  ${MUJOCO_LIB}
  ${GLFW_LIBRARIES}
)

# Executable node
add_executable(robot_state_visualization_node
  src/robot_state_visualization_node.cpp
)
ament_target_dependencies(robot_state_visualization_node
  rclcpp
  sensor_msgs
  ocs2_msgs
)
ament_target_dependencies(robot_state_visualization_lib
  rclcpp
  sensor_msgs
)

target_link_libraries(robot_state_visualization_node
  robot_state_visualization_lib
)

# Install targets
install(TARGETS
  robot_state_visualization_lib
  robot_state_visualization_node
  EXPORT export_robot_state_visualization_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_robot_state_visualization_lib HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
