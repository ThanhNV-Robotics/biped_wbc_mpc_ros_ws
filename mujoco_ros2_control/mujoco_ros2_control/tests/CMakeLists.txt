find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest(test_plugin test_plugin.cpp)
target_link_libraries(test_plugin
  mujoco_ros2_control
  rclcpp::rclcpp
)
target_include_directories(test_plugin SYSTEM PRIVATE
  ${MUJOCO_INCLUDE_DIR}
  ${MUJOCO_SIMULATE_DIR}
)
target_compile_options(test_plugin PRIVATE ${MUJOCO_SILENCE_COMPILER_WARNINGS})

ament_add_gtest(test_headless_init test_headless_init.cpp)
target_link_libraries(test_headless_init
  mujoco_ros2_control
  rclcpp::rclcpp
  hardware_interface::hardware_interface
)
target_include_directories(test_headless_init SYSTEM PRIVATE
  ${MUJOCO_INCLUDE_DIR}
  ${MUJOCO_SIMULATE_DIR}
)
target_compile_options(test_headless_init PRIVATE ${MUJOCO_SILENCE_COMPILER_WARNINGS})

ament_add_pytest_test(test_urdf_to_mujoco_utils test_urdf_to_mujoco_utils.py)
