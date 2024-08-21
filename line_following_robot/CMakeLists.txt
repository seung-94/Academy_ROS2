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
add_executable(line_following_robot src/line_following.cpp)
ament_target_dependencies(line_following_robot rclcpp sensor_msgs cv_bridge OpenCV std_msgs geometry_msgs rclcpp_action)

target_link_libraries(line_following_robot
  ${OpenCV_LIBRARIES}
)

# Install
install(TARGETS
  line_following_robot
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
