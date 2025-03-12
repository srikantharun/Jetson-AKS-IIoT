cmake_minimum_required(VERSION 3.8)
project(siwvs_bringup)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install worlds directory
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz configuration
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
