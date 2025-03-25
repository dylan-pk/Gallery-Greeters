cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(obstacle_detection src/obstacle_detection.cpp)

ament_target_dependencies(obstacle_detection rclcpp sensor_msgs)


install(TARGETS
  obstacle_detection
  DESTINATION lib/${PROJECT_NAME})

ament_package()
