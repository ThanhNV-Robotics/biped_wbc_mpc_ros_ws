cmake_minimum_required(VERSION 3.8)
project(control_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Find mujoco lib
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

# Action interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveOneJoint.action"
  "action/InitialStanding.action"
  "action/StandingBalance.action"
)

# Build the action server library
add_library(action_server SHARED
  src/control_actions_server.cpp
  src/move_one_joint_callbacks.cpp
  src/initial_standing_callbacks.cpp
  src/computation_utils.cpp
  src/standing_balance_callbacks.cpp
  src/standing_balance_control.cpp
)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS} 
)
target_compile_definitions(action_server
  PRIVATE "CONTROL_ACTIONS_CPP_BUILDING_DLL"
)
ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  trajectory_msgs
  sensor_msgs
)
add_dependencies(action_server ${PROJECT_NAME}__cpp)

# Build the standalone executable node
add_executable(control_actions_server_node src/control_actions_server_node.cpp)
target_link_libraries(control_actions_server_node
  action_server
  ${MUJOCO_LIB}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

target_include_directories(control_actions_server_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  ${MUJOCO_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS} 
)
add_dependencies(control_actions_server_node ${PROJECT_NAME}__cpp)

install(TARGETS
  action_server
  control_actions_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

# Register as a component and executable
rclcpp_components_register_node(action_server
  PLUGIN "control_actions_cpp::ControlActionServer"
  EXECUTABLE control_actions_server
)

# Install headers (if you have public headers)
install(DIRECTORY include/
  DESTINATION include
)

# Testing and linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()