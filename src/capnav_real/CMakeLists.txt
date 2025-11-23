cmake_minimum_required(VERSION 3.8)
project(capnav_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/yolo_sam_detector.py
  nodes/dinov2_clip_extractor.py
  nodes/instance_clusterer.py
  nodes/attribute_extractor.py
  nodes/map_builder.py
  nodes/language_parser.py
  nodes/goal_verifier.py
  nodes/capnav_navigator.py
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
