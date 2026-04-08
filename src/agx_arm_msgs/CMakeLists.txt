cmake_minimum_required(VERSION 3.5)
project(agx_arm_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

# Declare the message files
add_message_files(
  FILES
  AgxArmStatus.msg
  GripperStatus.msg
  HandStatus.msg
  HandCmd.msg
  HandPositionTimeCmd.msg
  MoveMITMsg.msg
)

# Generate added messages with dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)