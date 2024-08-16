cmake_minimum_required(VERSION 3.8)
project(project_14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# include directories
include_directories(include)

# add executable
add_executable(line_follower src/line_follower.cpp)
ament_target_dependencies(line_follower rclcpp geometry_msgs sensor_msgs cv_bridge OpenCV)

# install
install(TARGETS
  line_follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
