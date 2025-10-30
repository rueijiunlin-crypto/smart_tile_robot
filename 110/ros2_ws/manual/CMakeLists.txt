cmake_minimum_required(VERSION 3.8)
project(manual)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python executables
install(PROGRAMS 
  src/manual.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

