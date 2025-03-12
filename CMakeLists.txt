cmake_minimum_required(VERSION 3.8)
project(jetson_aks_iiot_workspace)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Add all subdirectories to build
add_subdirectory(k3s-siwvs/ros2/siwvs_msgs)
add_subdirectory(k3s-siwvs/ros2/siwvs_control)
add_subdirectory(k3s-siwvs/ros2/siwvs_bringup)
add_subdirectory(k3s-siwvs/gazebo/plugins/weight_sensor_plugin)
