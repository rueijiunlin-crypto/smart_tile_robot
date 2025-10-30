cmake_minimum_required(VERSION 3.5)
project(top_base_notify)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# Generate custom message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Move.msg"
  DEPENDENCIES std_msgs
)

# Install Python modules
install(PROGRAMS 
  src/top_base_noti.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclpy std_msgs geometry_msgs rosidl_default_runtime)
ament_package()

