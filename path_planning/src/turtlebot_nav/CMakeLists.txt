cmake_minimum_required(VERSION 3.8)
project(turtlebot_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add executable
add_executable(nav_to_goal src/nav_to_goal.cpp)
add_executable(nav_to_goal2 src/nav_to_goal2.cpp)
add_executable(path_to_goal src/path_to_goal.cpp)
add_executable(path_to_goal2 src/path_to_goal2.cpp)
add_executable(path_to_goal3 src/path_to_goal3.cpp)
add_executable(path_to_goal4 src/path_to_goal4.cpp)
add_executable(path_subscriber src/path_subscriber.cpp)
add_executable(goal_publisher src/goal_publisher.cpp)

# Link dependencies
ament_target_dependencies(nav_to_goal
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_target_dependencies(nav_to_goal2
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_target_dependencies(path_to_goal
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_target_dependencies(path_to_goal2
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_target_dependencies(path_to_goal3
  rclcpp
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(path_to_goal4
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

target_link_libraries(path_to_goal4
  yaml-cpp
  ${OpenCV_LIBS}
)

target_include_directories(path_to_goal4 PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(path_subscriber
  rclcpp
  nav_msgs
)

ament_target_dependencies(goal_publisher 
rclcpp geometry_msgs
)

# Install the executable
install(TARGETS
  nav_to_goal
  nav_to_goal2
  path_to_goal
  path_to_goal2
  path_to_goal3
  path_to_goal4
  path_subscriber
  goal_publisher 
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

