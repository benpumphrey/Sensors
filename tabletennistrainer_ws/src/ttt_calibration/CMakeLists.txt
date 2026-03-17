cmake_minimum_required(VERSION 3.8)
project(ttt_calibration)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 1. Original TF Broadcaster (Keep this for manual overrides/robot base)
add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp)
ament_target_dependencies(tf_broadcaster_node rclcpp geometry_msgs tf2_ros tf2_geometry_msgs)

# 2. NEW Auto-Calibrator Node
add_executable(table_calibrator_node src/table_calibrator_node.cpp)
target_include_directories(table_calibrator_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(table_calibrator_node rclcpp sensor_msgs geometry_msgs tf2_ros cv_bridge)
target_link_libraries(table_calibrator_node ${OpenCV_LIBRARIES})

install(TARGETS
  tf_broadcaster_node
  table_calibrator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
