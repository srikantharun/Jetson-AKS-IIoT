cmake_minimum_required(VERSION 3.8)
project(siwvs_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(siwvs_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Declare a C++ executable
add_executable(weight_sensor_node src/weight_sensor_node.cpp)
ament_target_dependencies(weight_sensor_node
  rclcpp
  siwvs_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Install executables
install(TARGETS
  weight_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

# Install Python nodes
install(PROGRAMS
  scripts/weight_sensor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
