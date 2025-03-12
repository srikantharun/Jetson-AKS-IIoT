cmake_minimum_required(VERSION 3.8)
project(ros2_workspace)

find_package(ament_cmake REQUIRED)

# Add all packages to build
add_subdirectory(siwvs_msgs)
add_subdirectory(siwvs_control)
add_subdirectory(siwvs_bringup)

ament_package()
