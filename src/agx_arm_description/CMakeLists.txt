cmake_minimum_required(VERSION 3.5)
project(agx_arm_description)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch meshes rviz urdf mujoco_model
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)