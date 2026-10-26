cmake_minimum_required(VERSION 3.8)
project(biped_wbc_core)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # Pinocchio / TSID / Eigen are very slow without optimization
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Eigen3: math everywhere. pinocchio (4.x, apt ros-humble-pinocchio): rigid-body kinematics /
# dynamics. tsid (1.x, apt ros-humble-tsid): task-space inverse dynamics, links pinocchio and
# eiquadprog. yaml-cpp: gain files (common/utils.cpp).
set(THIS_PACKAGE_DEPENDS
  Eigen3
  pinocchio
  tsid
  yaml-cpp
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

# One library with everything; add new .cpp files here
add_library(${PROJECT_NAME} SHARED
  src/common/utils.cpp
  src/QP_tsid/QP_tsid.cpp
  src/planner/LIPTaskPlanner.cpp
  src/state_estimation/contact_fusion.cpp
  src/state_estimation/kalman_filter.cpp
  src/state_estimation/StateEstimator.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# Headers are included as <biped_wbc/...>, in this package and by users of it
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Imported targets carry their include dirs, compile definitions and transitive libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  tsid::tsid
  yaml-cpp
)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Visual tests (MuJoCo window, test/): only the tests use MuJoCo and GLFW, the library does not.
# Switch off with: colcon build --cmake-args -DBUILD_VISUAL_TESTS=OFF
option(BUILD_VISUAL_TESTS "Build the MuJoCo visual tests in test/" ON)
if(BUILD_VISUAL_TESTS)
  find_package(mujoco_vendor REQUIRED)  # target mujoco::mujoco
  find_package(glfw3 REQUIRED)          # target glfw

  # LIPTaskPlanner in real time: ros2 run biped_wbc_core test_lip_task_planner [yaml] [xml]
  add_executable(test_lip_task_planner test/test_LIPTaskPlanner/testLIPTaskPlanner.cpp)
  target_link_libraries(test_lip_task_planner ${PROJECT_NAME} mujoco::mujoco glfw)
  # default input files: the scene next to the test, the planner settings of biped_bringup
  target_compile_definitions(test_lip_task_planner PRIVATE
    LIP_MODEL_XML="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/test/LIP_model.xml"
    TASK_PLANNER_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../biped_bringup/config/task_planner.yaml")
  install(TARGETS test_lip_task_planner DESTINATION lib/${PROJECT_NAME})
  install(FILES test/test_LIPTaskPlanner/LIP_model.xml DESTINATION share/${PROJECT_NAME}/test)
endif()

# Other packages (e.g. a ros2_control controller in biped_controllers) use it with
#   find_package(biped_wbc_core REQUIRED)
#   target_link_libraries(<target> biped_wbc_core::biped_wbc_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_package()
