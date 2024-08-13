// 라인 트레이싱 로봇

cmake_minimum_required(VERSION 3.5)
project(line_follower)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(line_follower_node src/line_follower_node.cpp)

ament_target_dependencies(line_follower_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  line_follower_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
