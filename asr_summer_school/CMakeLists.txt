cmake_minimum_required(VERSION 3.8)
project(asr_summer_school)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(frontier_detection
  src/frontier_detection.cpp
)
target_include_directories(frontier_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frontier_detection
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)
set_target_properties(frontier_detection PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(frontier_detection_node SHARED
  src/frontier_detection_node.cpp
)
target_link_libraries(frontier_detection_node frontier_detection)
ament_target_dependencies(frontier_detection_node rclcpp rclcpp_components nav_msgs geometry_msgs visualization_msgs)
rclcpp_components_register_node(frontier_detection_node
  PLUGIN "FrontierDetectionNode"
  EXECUTABLE frontier_detection_node_exe
)

ament_python_install_package(${PROJECT_NAME})

# Every runnable Python node must be listed here or `ros2 run` will not find
# it: this package is ament_cmake, so ament_python_install_package installs the
# importable module but not the executables.
install(PROGRAMS
  ${PROJECT_NAME}/example_nav_to_pose.py
  ${PROJECT_NAME}/mission_control.py
  ${PROJECT_NAME}/scan_preprocess.py
  ${PROJECT_NAME}/tag_manager.py
  ${PROJECT_NAME}/score_report.py
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS frontier_detection frontier_detection_node
  EXPORT export_frontier_detection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config test worlds models
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

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

ament_package()
