cmake_minimum_required(VERSION 3.8)
project(line_following_robot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(line_following_robot src/line_following.cpp src/other_source_files.cpp)

# Link dependencies
ament_target_dependencies(line_following_robot
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(line_following_robot
  ${OpenCV_LIBRARIES}
)

# Install the executable
install(TARGETS
  line_following_robot
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
