cmake_minimum_required(VERSION 3.8)

project(agx_arm_description)

find_package(ament_cmake REQUIRED)

# Install resources (URDF/meshes/config/launch) from subdirectories
install(DIRECTORY 
  nero  
  piper
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
