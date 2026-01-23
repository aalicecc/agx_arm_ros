# AgileX Robotic Arm ROS2 Driver (Humble)

[中文](./README_ZH.md)

## Overview

The **AgileX Robotic Arm ROS2 Driver Package** provides a complete ROS2 driver for AgileX series robotic arms.

---

## Quick Start

### Dependencies

   ```bash
   pip3 install python-can>=4.3.1 scipy numpy

   # CAN tools
   sudo apt update && sudo apt install can-utils ethtool

   # ROS2 dependencies
   sudo apt install ros-$ROS_DISTRO-ros2-control
   sudo apt install ros-$ROS_DISTRO-ros2-controllers
   sudo apt install ros-$ROS_DISTRO-controller-manager
   sudo apt install ros-$ROS_DISTRO-topic-tools

   # Optional dependencies
   sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
   sudo apt install ros-$ROS_DISTRO-robot-state-publisher
   sudo apt install ros-$ROS_DISTRO-xacro
   ```

### Installation

#### ROS2 Driver Installation

0. First complete all steps in [Dependencies](#dependencies)

1. Create a ROS workspace and navigate to the src directory:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```

2. Clone the required repositories:
   ```bash
   # TODO
   git clone <repository-url> nero
   ```

3. Build the workspace:
   ```bash
   cd agx_arm_ros
   colcon build
   source install/setup.bash
   ```

#### Python SDK Installation

   ```bash
   # TODO
   pip3 install sdk
   ```
   
---

## ROS2 Driver Usage

### Activate CAN Module
See: [CAN_USER](./docs/CAN_USER.md)

### Launch Files

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py
```

### Launch Parameters

| Parameter | Default | Description | Options |
|-----------|---------|-------------|---------|
| `can_port` | `can0` | CAN port name | - |
| `arm_type` | `piper_h` | Robotic arm type | `piper`, `piper_h`, `piper_l`, `piper_x`, `nero` |
| `effector_type` | `none` | End-effector type | `none`, `agx_gripper`, `revo2` |
| `auto_enable` | `True` | Auto enable arm on startup | `True`, `False` |
| `installation_pos` | `horizontal` | Arm installation position | `horizontal`, `left`, `right` |
| `speed_percent` | `100` | Movement speed percentage | `0-100` |
| `pub_rate` | `200` | Feedback publishing rate (Hz) | - |
| `enable_timeout` | `5.0` | Enable/disable timeout (seconds) | - |
| `log_level` | `info` | Logging level | `debug`, `info`, `warn`, `error`, `fatal` |

---

## ROS2 Topics and Services

### Feedback Topics (Published)

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/feedback/joint_states` | `sensor_msgs/JointState` | Current joint positions (rad) |
| `/feedback/end_pose` | `geometry_msgs/PoseStamped` | Current end-effector pose |
| `/feedback/arm_status` | `agx_arm_msgs/AgxArmStatus` | Arm status (control mode, motion status, errors, etc.) |
| `/feedback/arm_ctrl_states` | `sensor_msgs/JointState` | Joint control states (Piper only) |
| `/feedback/gripper_status` | `agx_arm_msgs/GripperStatus` | Gripper status |
| `/feedback/hand_status` | `agx_arm_msgs/HandStatus` | Dexterous hand status |

### Control Topics (Subscribed)

| Topic | Message Type | Description | Supported Arms |
|-------|--------------|-------------|----------------|
| `/control/move_j` | `sensor_msgs/JointState` | Joint position control | All |
| `/control/move_p` | `geometry_msgs/PoseStamped` | End-effector pose control | All |
| `/control/move_l` | `geometry_msgs/PoseStamped` | Linear motion control | Piper only |
| `/control/move_c` | `agx_arm_msgs/TriplePose` | Circular motion control | Piper only |
| `/control/move_js` | `sensor_msgs/JointState` | MIT mode joint position motion | Piper only |
| `/control/move_mit` | `sensor_msgs/JointState` | MIT mode control | Piper only |
| `/control/gripper` | `agx_arm_msgs/GripperCmd` | Gripper control | If gripper configured |
| `/control/hand` | `agx_arm_msgs/HandCmd` | Dexterous hand control (position/speed/current) | If hand configured |
| `/control/hand_position_time` | `agx_arm_msgs/HandPositionTimeCmd` | Dexterous hand position+time control | If hand configured |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/enable_agx_arm` | `std_srvs/SetBool` | Enable (True) / Disable (False) robotic arm |
| `/move_home` | `std_srvs/Empty` | Move all joints to zero position |
| `/exit_teach_mode` | `std_srvs/Empty` | Exit teach mode and reset arm |

---

## Important Notes

### CAN Communication

- **CAN module must be activated before use**
- **Use correct baud rate**: 1000000 bps
- **If messages show `SendCanMessage failed`**, please check CAN connection

### Critical Safety Warnings

- **Always maintain a safe distance from the moving robotic arm during operation.** Entering the robot's workspace while it is active can result in severe injury.
- **Numerical solutions near kinematic singularities may cause sudden, large joint movements.** The arm's motion can become unpredictable, please maintain a safe distance.
- **The high-speed response MIT mode is extremely dangerous.** Use with caution and only when absolutely necessary.
