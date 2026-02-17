cmake_minimum_required(VERSION 3.8)
project(ttt_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ttt_msgs REQUIRED)

# Find standard OpenCV
find_package(OpenCV REQUIRED)

# -------------------------------------------------------------
# 1. Ball Detector Node (Pure OpenCV)
# -------------------------------------------------------------
add_executable(vision_node src/vision_node.cpp)

# Link standard OpenCV include directories and libraries
target_include_directories(vision_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_node ${OpenCV_LIBS})

# Link ROS 2 packages
ament_target_dependencies(vision_node
  ttt_msgs
  rclcpp
  sensor_msgs
  cv_bridge
  ttt_msgs
)

# -------------------------------------------------------------
# Install Executables
# -------------------------------------------------------------
install(TARGETS
  vision_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files/directories if you add them later
# install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
