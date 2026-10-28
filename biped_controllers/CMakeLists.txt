cmake_minimum_required(VERSION 3.8)
project(biped_controllers)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # the controller runs Eigen code in the real-time loop: optimized, no Eigen debug asserts
  # (same default as biped_wbc_core; override with --cmake-args -DCMAKE_BUILD_TYPE=Debug)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  humanoid_mpc_msgs  # motion commands (keyboard / joystick)
  nav_msgs           # simulator ground truth (use_ground_truth)
  ament_index_cpp  # package share directories (URDF, WBC yaml)
  biped_wbc_core   # ROS-free WBC algorithms (state estimator, contact fusion, TSID)
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/joint_jogging_controller/joint_jogging_controller.cpp
  src/wbc_controller/wbc_controller.cpp
  src/wbc_controller/controller_params.cpp  # parameters shared by the biped controllers
  src/wbc_controller/joint_map.cpp          # joint order ros2_control <-> Pinocchio
  src/wbc_controller/robot_interface.cpp    # ros2_control interfaces: claim, bind, read, write, hold
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface biped_controllers.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_package()
