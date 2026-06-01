cmake_minimum_required(VERSION 3.16)
project(state_estimation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Find dependencies ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ── Library ────────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/StateEstimation.cpp
  src/LinearKalmanFilter.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  geometry_msgs
  realtime_tools
  hardware_interface
  ocs2_centroidal_model
  ocs2_legged_robot
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── Install ────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# ── Export ─────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  realtime_tools
  hardware_interface
  ocs2_centroidal_model
  ocs2_legged_robot
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  tf2
  tf2_ros
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
