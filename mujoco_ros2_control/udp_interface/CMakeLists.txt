cmake_minimum_required(VERSION 3.16)
project(udp_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(udp_bridge_node
  src/UdpSocket.cpp
  src/udp_bridge_node.cpp
)

target_include_directories(udp_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(udp_bridge_node
  rclcpp
  sensor_msgs
)

install(TARGETS udp_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
