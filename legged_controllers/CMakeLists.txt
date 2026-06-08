cmake_minimum_required(VERSION 3.16)
project(legged_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# TODO(step2): uncomment as each dependency is ported to ROS2
# find_package(legged_common REQUIRED)
# find_package(legged_interface REQUIRED)
# find_package(legged_estimation REQUIRED)
# find_package(legged_wbc REQUIRED)
# find_package(ocs2_legged_robot_ros REQUIRED)
# find_package(ocs2_self_collision_visualization REQUIRED)
# find_package(angles REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/LeggedController.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(controller_interface legged_controllers_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES legged_controllers_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
)

ament_package()
