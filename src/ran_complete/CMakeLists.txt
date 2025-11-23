cmake_minimum_required(VERSION 3.8)
project(ran_complete)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (all core nodes)
install(PROGRAMS
  nodes/ran_perception_node.py
  nodes/ran_mapping_node.py
  nodes/ran_hierarchical_verifier.py
  nodes/ran_navigator.py
  nodes/ran_safety_monitor.py
  scripts/download_models.sh
  scripts/test_system.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and configs
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
