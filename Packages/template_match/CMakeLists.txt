cmake_minimum_required(VERSION 3.5)
project(template_match)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install the resource directory to the 'share' folder
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)
  
# If your Python script is part of the package and should be installed as a Python module, use this:
install(PROGRAMS
  template_match/template_match.py  # Specify the full path to your Python script
  DESTINATION lib/${PROJECT_NAME}   # Ptlace in lib/${PROJECT_NAME} for ROS2 scripts
)

ament_package()
