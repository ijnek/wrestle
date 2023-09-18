cmake_minimum_required(VERSION 3.8)
project(wrestle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch plotjuggler pos rviz urdf
  DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS
  scripts/head_skill.py
  scripts/robot_detection.py
  scripts/motion_manager.py
  scripts/arm_provider.py
  scripts/crouch.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
