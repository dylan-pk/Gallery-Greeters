cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(obstacle_detection src/obstacle_detection.cpp)

ament_target_dependencies(obstacle_detection rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)


install(TARGETS
  obstacle_detection
  DESTINATION lib/${PROJECT_NAME})

ament_package()
