cmake_minimum_required(VERSION 3.8)
project(navigation_control)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 串口库
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

set(dependencies
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Include directories
include_directories(
  include
  ${SERIAL_INCLUDE_DIRS}
)

# ============ 核心节点 ============

# Wheel Odometry Node (轮式里程计 - 发布/odom和TF)
add_executable(wheel_odometry_node
  src/wheel_odometry_node.cpp
)
ament_target_dependencies(wheel_odometry_node ${dependencies})

# Serial Communication Node (串口通信 - 接收下位机数据)
add_executable(serial_communication
  src/serial_communication.cpp
)
ament_target_dependencies(serial_communication ${dependencies})
target_link_libraries(serial_communication ${SERIAL_LIBRARIES})

# Serial Data Publisher Node (速度命令转协议 - 发送到下位机)
add_executable(serial_data_publisher
  src/serial_data_publisher.cpp
)
ament_target_dependencies(serial_data_publisher ${dependencies})

# Scan Filter Node (激光过滤 - 过滤机器人本体)
add_executable(scan_filter_node
  src/scan_filter_node.cpp
)
ament_target_dependencies(scan_filter_node ${dependencies})

# Install executables
install(TARGETS
  wheel_odometry_node
  serial_communication
  serial_data_publisher
  scan_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/astar_planner.py
  scripts/simple_goal_controller.py
  scripts/map_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install maps directory
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
