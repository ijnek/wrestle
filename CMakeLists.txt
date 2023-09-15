cmake_minimum_required(VERSION 3.8)
project(wrestle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  geometry_msgs
  lifecycle_msgs
  nao_lola_sensor_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
)

# Build MotionManagerNode
add_library(motion_manager_node SHARED
  src/motion_manager_node.cpp)
target_include_directories(motion_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(motion_manager_node
  PLUGIN "wrestle::MotionManagerNode"
  EXECUTABLE motion_manager)

ament_target_dependencies(motion_manager_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_motion_manager HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS motion_manager_node
  EXPORT export_motion_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY launch plotjuggler pos rviz urdf
  DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS
  scripts/head_skill.py
  scripts/robot_detection.py
  scripts/motion_manager.py
  scripts/arm_provider.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
