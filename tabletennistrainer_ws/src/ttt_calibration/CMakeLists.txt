cmake_minimum_required(VERSION 3.8)
project(ttt_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# ArUco Table Calibration Node
add_executable(arcuo_calibrate_camera src/arcuo_calibrate_camera.cpp)

ament_target_dependencies(arcuo_calibrate_camera
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(arcuo_calibrate_camera
  ${OpenCV_LIBS}
  yaml-cpp
)

# TF Broadcaster Node (keep existing)
add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp)

ament_target_dependencies(tf_broadcaster_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install executables
install(TARGETS
  arcuo_calibrate_camera
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