cmake_minimum_required(VERSION 3.8)
project(p_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(p_controller src/p_controller.cpp src/obstacle_avoidance.cpp)

add_executable(test_path_publisher src/test_path_publisher.cpp)

add_library(obstacle_avoidance src/obstacle_avoidance.cpp)

ament_target_dependencies(p_controller rclcpp visualization_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

ament_target_dependencies(test_path_publisher rclcpp nav_msgs geometry_msgs tf2)

ament_target_dependencies(obstacle_avoidance rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

target_link_libraries(p_controller obstacle_avoidance)

install(TARGETS
  p_controller
  test_path_publisher
  # obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME})

ament_package()
