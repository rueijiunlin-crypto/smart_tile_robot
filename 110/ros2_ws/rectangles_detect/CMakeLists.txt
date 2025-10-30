cmake_minimum_required(VERSION 3.5)
project(rectangles_detect)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

install(PROGRAMS
  src/rectangles_detect2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
