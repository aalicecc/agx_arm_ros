cmake_minimum_required(VERSION 3.5)
project(agx_arm_ctrl)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_srvs
  agx_arm_msgs
)

catkin_python_setup()

catkin_package()

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "*.launch"
)

# 安装可执行节点
catkin_install_python(PROGRAMS
  agx_arm_ctrl/agx_arm_ctrl_single_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

