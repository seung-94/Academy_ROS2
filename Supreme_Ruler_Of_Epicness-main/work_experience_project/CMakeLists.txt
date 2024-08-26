cmake_minimum_required(VERSION 3.8)
project(work_experience_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Custom service
# find_package(rosidl_default_generators REQUIRED)

# Generate the custom service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/WaypointService.srv"
)

set(dependencies "rclcpp" "geometry_msgs" "nav2_msgs" "tf2" "tf2_ros" "tf2_geometry_msgs" "example_interfaces" "std_msgs" "std_srvs")

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Node executables
add_executable(waypoints_service_server src/waypoints_service_server.cpp)
ament_target_dependencies(waypoints_service_server rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs rosidl_default_runtime ${dependencies})
target_link_libraries(waypoints_service_server "${cpp_typesupport_target}")

add_executable(waypoints_follower_client src/waypoints_follower_client.cpp)
ament_target_dependencies(waypoints_follower_client rclcpp geometry_msgs nav2_msgs tf2 tf2_ros tf2_geometry_msgs rosidl_default_runtime ${dependencies})
target_link_libraries(waypoints_follower_client "${cpp_typesupport_target}")

add_executable(current_position_publisher src/current_position_publisher.cpp)
ament_target_dependencies(current_position_publisher rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs rosidl_default_runtime ${dependencies})
target_link_libraries(current_position_publisher "${cpp_typesupport_target}")

add_executable(target_position_subscriber src/target_position_subscriber.cpp)
ament_target_dependencies(target_position_subscriber rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs rosidl_default_runtime ${dependencies})
target_link_libraries(target_position_subscriber "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  waypoints_service_server
  waypoints_follower_client
  current_position_publisher
  target_position_subscriber
  DESTINATION lib/${PROJECT_NAME})

# Install service files
install(DIRECTORY srv/
DESTINATION share/${PROJECT_NAME}/srv)

ament_package()