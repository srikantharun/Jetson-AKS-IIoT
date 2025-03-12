cmake_minimum_required(VERSION 3.8)
project(siwvs_gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(siwvs_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_PROTO_INCLUDE_DIRS}
)

# Link directories
link_directories(${GAZEBO_LIBRARY_DIRS})

# List sources for the weight sensor plugin
set(weight_sensor_plugin_src
  src/weight_sensor_plugin.cpp
)

# Create the weight sensor plugin library
add_library(WeightSensorPlugin SHARED ${weight_sensor_plugin_src})
ament_target_dependencies(WeightSensorPlugin
  rclcpp
  gazebo_ros
  siwvs_msgs
  std_msgs
)
target_link_libraries(WeightSensorPlugin
  ${GAZEBO_LIBRARIES}
)

# Install the plugin library
install(TARGETS
  WeightSensorPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(WeightSensorPlugin)
ament_export_dependencies(
  rclcpp
  gazebo_ros
  siwvs_msgs
  std_msgs
)

ament_package()
