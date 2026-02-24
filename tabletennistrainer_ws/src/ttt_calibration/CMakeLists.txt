cmake_minimum_required(VERSION 3.8)
project(ttt_calibration)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# TF Broadcaster Node
add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp)
ament_target_dependencies(tf_broadcaster_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()