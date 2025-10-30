cmake_minimum_required(VERSION 3.5)
project(allnode)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)





# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



# Linting configuration
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

ament_package()
