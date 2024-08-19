cmake_minimum_required(VERSION 3.5)
project(line_follower)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare C++ executable
add_executable(line_follower src/line_follower.cpp)

# Specify libraries to link executable targets against
ament_target_dependencies(line_follower rclcpp std_msgs geometry_msgs)

# Install targets
install(TARGETS
  line_follower
  DESTINATION lib/${PROJECT_NAME}
)

# Register the package
ament_package()
