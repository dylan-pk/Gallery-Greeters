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

add_executable(p_controller src/p_controller.cpp)

ament_target_dependencies(p_controller rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)


install(TARGETS
  p_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
