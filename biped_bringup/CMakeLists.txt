cmake_minimum_required(VERSION 3.8)
project(biped_bringup)

# Launch files and system configuration only (no C++): how the robot / simulation is started.
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
