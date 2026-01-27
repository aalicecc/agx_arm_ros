# AgileX Robotic Arm ROS2 Driver (Humble)

[中文](./README_ZH.md)

## Overview

This driver package provides full ROS2 interface support for AgileX series robotic arms (Piper, Nero, etc.).

---

## Quick Start

### 1. Install Dependencies

```bash
# Python dependencies
pip3 install "python-can>=4.3.1" scipy numpy

# CAN tools
sudo apt update && sudo apt install can-utils ethtool

# ROS2 dependencies
sudo apt install ros-$ROS_DISTRO-ros2-control \
                 ros-$ROS_DISTRO-ros2-controllers \
                 ros-$ROS_DISTRO-controller-manager \
                 ros-$ROS_DISTRO-topic-tools

# Optional dependencies (for visualization and simulation)
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui \
                 ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-xacro
```

### 2. Install Python SDK

```bash
git clone https://github.com/aalicecc/agx_arm_ros.git
cd pyAgxArm
pip3 install .
```

### 3. Install ROS2 Driver

```bash
# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone repository
git clone http://10.20.0.138:8000/aalicecc/agx_arm_ros.git

# Build
cd ~/catkin_ws
colcon build
source install/setup.bash
```

---

## Usage

### Activate CAN Module

CAN module must be activated before use. See: [CAN Configuration Guide](./docs/CAN_USER.md)

### Launch Driver

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py
```

### Launch Parameters

| Parameter | Default | Description | Options |
|-----------|---------|-------------|---------|
| `can_port` | `can0` | CAN port | - |
| `arm_type` | `piper` | Arm model | `piper`, `piper_h`, `piper_l`, `piper_x`, `nero` |
| `effector_type` | `none` | End-effector type | `none`, `agx_gripper`, `revo2` |
| `auto_enable` | `True` | Auto enable on startup | `True`, `False` |
| `installation_pos` | `horizontal` | Mount orientation (Piper series only) | `horizontal`, `left`, `right` |
| `payload` | `empty` | Payload config (AgxGripper only) | `full`, `half`, `empty` |
| `speed_percent` | `100` | Motion speed (%) | `0-100` |
| `pub_rate` | `200` | Status publish rate (Hz) | - |
| `enable_timeout` | `5.0` | Enable timeout (seconds) | - |
| `tcp_offset` | `[0,0,0,0,0,0]` | TCP offset [x,y,z,rx,ry,rz] | - |
| `log_level` | `info` | Log level | `debug`, `info`, `warn`, `error`, `fatal` |

---

## ROS2 Interface

### Feedback Topics

| Topic | Message Type | Description | Condition |
|-------|--------------|-------------|-----------|
| `/feedback/joint_states` | `sensor_msgs/JointState` | Joint states | Always available |
| `/feedback/tcp_pose` | `geometry_msgs/PoseStamped` | TCP pose | Always available |
| `/feedback/arm_status` | `agx_arm_msgs/AgxArmStatus` | Arm status | Always available |
| `/feedback/arm_ctrl_states` | `sensor_msgs/JointState` | Joint control states | Piper series |
| `/feedback/gripper_status` | `agx_arm_msgs/GripperStatus` | Gripper status | AgxGripper configured |
| `/feedback/hand_status` | `agx_arm_msgs/HandStatus` | Dexterous hand status | Revo2 configured |

#### Joint States Details (`/feedback/joint_states`)

This topic contains combined joint states for the arm and end-effector:

**Arm Joints** (`joint1` ~ `joint6`)
| Field | Description |
|-------|-------------|
| `position` | Joint angle (rad) |
| `velocity` | 0.0 |
| `effort` | 0.0 |

**Gripper Joint** (`gripper`, requires `effector_type=agx_gripper`)
| Field | Description |
|-------|-------------|
| `position` | Gripper width (m) |
| `velocity` | 0.0 |
| `effort` | 0.0 |

**Dexterous Hand Joints** (requires `effector_type=revo2`)

Joint naming convention: `{hand}_f_joint{finger}_[segment]`
- `l` = left hand, `r` = right hand
- `joint1` = thumb, `joint2` ~ `joint5` = index to pinky
- `_1` = base, `_2` = tip (thumb only has two segments)

Example: `l_f_joint1_1` represents left hand thumb base joint

Full joint name list:
- Left hand: `l_f_joint1_1`, `l_f_joint1_2`, `l_f_joint2`, `l_f_joint3`, `l_f_joint4`, `l_f_joint5`
- Right hand: `r_f_joint1_1`, `r_f_joint1_2`, `r_f_joint2`, `r_f_joint3`, `r_f_joint4`, `r_f_joint5`

### Control Topics

| Topic | Message Type | Description | Condition |
|-------|--------------|-------------|-----------|
| `/control/joint_states` | `sensor_msgs/JointState` | Joint control (with end-effector) | Always available |
| `/control/move_j` | `sensor_msgs/JointState` | Joint control motion | Always available |
| `/control/move_p` | `geometry_msgs/PoseStamped` | Cartesian space motion | Always available |
| `/control/move_l` | `geometry_msgs/PoseStamped` | Linear motion | Piper series |
| `/control/move_c` | `geometry_msgs/PoseArray` | Circular motion | Piper series |
| `/control/move_js` | `sensor_msgs/JointState` | MIT mode joint motion | Piper series |
| `/control/move_mit` | `agx_arm_msgs/MoveMITMsg` | MIT torque control | Piper series |
| `/control/gripper` | `agx_arm_msgs/GripperCmd` | Gripper control | AgxGripper configured |
| `/control/hand` | `agx_arm_msgs/HandCmd` | Dexterous hand control | Revo2 configured |
| `/control/hand_position_time` | `agx_arm_msgs/HandPositionTimeCmd` | Hand position-time control | Revo2 configured |

### Services

| Service | Type | Description | Condition |
|---------|------|-------------|-----------|
| `/enable_agx_arm` | `std_srvs/SetBool` | Enable/disable arm | Always available |
| `/move_home` | `std_srvs/Empty` | Move to home position | Always available |
| `/exit_teach_mode` | `std_srvs/Empty` | Exit teach mode | Piper series |

---

## Control Examples

> **Note**: Replace `AGX_ARM_ROS_PATH` with the actual path to the `agx_arm_ros` package.

### Piper Arm

```bash
# Joint motion
ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
  "$(cat AGX_ARM_ROS_PATH/test/piper/test_move_j.yaml)" -1

# Cartesian space motion
ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped \
  "$(cat AGX_ARM_ROS_PATH/test/piper/test_move_p.yaml)" -1

# Linear motion
ros2 topic pub /control/move_l geometry_msgs/msg/PoseStamped \
  "$(cat AGX_ARM_ROS_PATH/test/piper/test_move_l.yaml)" -1

# Circular motion (start → middle → end)
ros2 topic pub /control/move_c geometry_msgs/msg/PoseArray \
  "$(cat AGX_ARM_ROS_PATH/test/piper/test_move_c.yaml)" -1

# Gripper control (width: 0.05m, force: 1.0N)
ros2 topic pub /control/gripper agx_arm_msgs/msg/GripperCmd \
  "{width: 0.05, force: 1.0}" -1
```

### Nero Arm

```bash
# Joint motion
ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
  "$(cat AGX_ARM_ROS_PATH/test/nero/test_move_j.yaml)" -1

# Cartesian space motion
ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped \
  "$(cat AGX_ARM_ROS_PATH/test/nero/test_move_p.yaml)" -1
```

### Service Calls

```bash
# Enable arm
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"

# Disable arm
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"

# Move to home position
ros2 service call /move_home std_srvs/srv/Empty

# Exit teach mode (Piper series)
ros2 service call /exit_teach_mode std_srvs/srv/Empty
```

### Status Subscription

```bash
# Joint states
ros2 topic echo /feedback/joint_states

# TCP pose
ros2 topic echo /feedback/tcp_pose

# Arm status
ros2 topic echo /feedback/arm_status

# Gripper status
ros2 topic echo /feedback/gripper_status
```

---

## Important Notes

### CAN Communication

- CAN module **must be activated** before use
- Baud rate: **1000000 bps**
- If `SendCanMessage failed` error occurs, check CAN connection

### ⚠️ Safety Warnings

- **Maintain safe distance**: Do not enter the arm's workspace during motion to avoid injury
- **Singularity risk**: Joints may move suddenly and significantly near kinematic singularities
- **MIT mode is dangerous**: High-speed response MIT mode is extremely hazardous, use with caution
