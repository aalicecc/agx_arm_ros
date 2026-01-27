# AgileX 机械臂 ROS2 驱动 (Humble)

[English](./README_EN.md)

## 概述

本驱动包为 AgileX 系列机械臂（Piper、Nero 等）提供完整的 ROS2 接口支持。

---

## 快速开始

### 1. 安装依赖

```bash
# Python 依赖
pip3 install python-can>=4.3.1 scipy numpy

# CAN 工具
sudo apt update && sudo apt install can-utils ethtool

# ROS2 依赖
sudo apt install ros-$ROS_DISTRO-ros2-control \
                 ros-$ROS_DISTRO-ros2-controllers \
                 ros-$ROS_DISTRO-controller-manager \
                 ros-$ROS_DISTRO-topic-tools

# 可选依赖（用于可视化和仿真）
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui \
                 ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-xacro
```

### 2. 安装 Python SDK

```bash
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
pip3 install .
```

### 3. 安装 ROS2 驱动

```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆仓库
git clone https://github.com/aalicecc/agx_arm_ros.git

# 编译
cd ~/catkin_ws
colcon build
source install/setup.bash
```

---

## 使用说明

### 激活 CAN 模块

使用前需先激活 CAN 模块，详见：[CAN 配置指南](./docs/CAN_USER.md)

### 启动驱动

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py
```

### 启动参数

| 参数 | 默认值 | 说明 | 可选值 |
|------|--------|------|--------|
| `can_port` | `can0` | CAN 端口 | - |
| `arm_type` | `piper` | 机械臂型号 | `piper`, `piper_h`, `piper_l`, `piper_x`, `nero` |
| `effector_type` | `none` | 末端执行器类型 | `none`, `agx_gripper`, `revo2` |
| `auto_enable` | `True` | 启动时自动使能 | `True`, `False` |
| `installation_pos` | `horizontal` | 安装方向 (Piper 系列专用) | `horizontal`, `left`, `right` |
| `payload` | `empty` | 负载配置 (AgxGripper 专用) | `full`, `half`, `empty` |
| `speed_percent` | `100` | 运动速度 (%) | `0-100` |
| `pub_rate` | `200` | 状态发布频率 (Hz) | - |
| `enable_timeout` | `5.0` | 使能超时 (秒) | - |
| `tcp_offset` | `[0,0,0,0,0,0]` | TCP 偏移 [x,y,z,rx,ry,rz] | - |
| `log_level` | `info` | 日志级别 | `debug`, `info`, `warn`, `error`, `fatal` |

---

## ROS2 接口

### 反馈话题

| 话题 | 消息类型 | 说明 | 适用条件 |
|------|----------|------|----------|
| `/feedback/joint_states` | `sensor_msgs/JointState` | 关节状态 | 始终可用 |
| `/feedback/tcp_pose` | `geometry_msgs/PoseStamped` | TCP 位姿 | 始终可用 |
| `/feedback/arm_status` | `agx_arm_msgs/AgxArmStatus` | 机械臂状态 | 始终可用 |
| `/feedback/arm_ctrl_states` | `sensor_msgs/JointState` | 关节控制状态 | Piper 系列 |
| `/feedback/gripper_status` | `agx_arm_msgs/GripperStatus` | 夹爪状态 | 配置 AgxGripper |
| `/feedback/hand_status` | `agx_arm_msgs/HandStatus` | 灵巧手状态 | 配置 Revo2 |

#### 关节状态详情 (`/feedback/joint_states`)

该话题包含机械臂和末端执行器的组合关节状态：

**机械臂关节** (`joint1` ~ `joint6`)
| 字段 | 说明 |
|------|------|
| `position` | 关节角度 (rad) |
| `velocity` | 0.0 |
| `effort` | 0.0 |

**夹爪关节** (`gripper`，需配置 `effector_type=agx_gripper`)
| 字段 | 说明 |
|------|------|
| `position` | 夹爪宽度 (m) |
| `velocity` | 0.0 |
| `effort` | 0.0 |

**灵巧手关节**（需配置 `effector_type=revo2`）

关节命名规则：`{手}_f_joint{指}_[段]`
- `l` = 左手，`r` = 右手
- `joint1` = 大拇指，`joint2` ~ `joint5` = 食指到小指
- `_1` = 指根，`_2` = 指尖（仅大拇指有两段）

示例：`l_f_joint1_1` 表示左手大拇指指根关节

完整关节名列表：
- 左手：`l_f_joint1_1`, `l_f_joint1_2`, `l_f_joint2`, `l_f_joint3`, `l_f_joint4`, `l_f_joint5`
- 右手：`r_f_joint1_1`, `r_f_joint1_2`, `r_f_joint2`, `r_f_joint3`, `r_f_joint4`, `r_f_joint5`

### 控制话题

| 话题 | 消息类型 | 说明 | 适用条件 |
|------|----------|------|----------|
| `/control/joint_states` | `sensor_msgs/JointState` | 关节控制（含末端执行器） | 始终可用 |
| `/control/move_j` | `sensor_msgs/JointState` | 关节控制运动 | 始终可用 |
| `/control/move_p` | `geometry_msgs/PoseStamped` | 笛卡尔空间运动 | 始终可用 |
| `/control/move_l` | `geometry_msgs/PoseStamped` | 直线运动 | Piper 系列 |
| `/control/move_c` | `geometry_msgs/PoseArray` | 圆弧运动 | Piper 系列 |
| `/control/move_js` | `sensor_msgs/JointState` | MIT 模式关节运动 | Piper 系列 |
| `/control/move_mit` | `agx_arm_msgs/MoveMITMsg` | MIT 力矩控制 | Piper 系列 |
| `/control/gripper` | `agx_arm_msgs/GripperCmd` | 夹爪控制 | 配置 AgxGripper |
| `/control/hand` | `agx_arm_msgs/HandCmd` | 灵巧手控制 | 配置 Revo2 |
| `/control/hand_position_time` | `agx_arm_msgs/HandPositionTimeCmd` | 灵巧手位置时间控制 | 配置 Revo2 |

### 服务

| 服务 | 类型 | 说明 | 适用条件 |
|------|------|------|----------|
| `/enable_agx_arm` | `std_srvs/SetBool` | 使能/失能机械臂 | 始终可用 |
| `/move_home` | `std_srvs/Empty` | 回零位 | 始终可用 |
| `/exit_teach_mode` | `std_srvs/Empty` | 退出示教模式 | Piper 系列 |

---

## 控制示例

> **注意**：以下示例中的 `AGX_ARM_ROS_PATH` 需替换为实际的 `agx_arm_ros` 功能包路径。

### Piper 机械臂

```bash
# 关节运动
ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
  "$(cat AGX_ARM_ROS_PATH/test/piper/test_move_j.yaml)" -1

# 笛卡尔空间运动
ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped \
  "$(cat AGX_ARM_ROS_PATH/test/piper/test_move_p.yaml)" -1

# 直线运动
ros2 topic pub /control/move_l geometry_msgs/msg/PoseStamped \
  "$(cat AGX_ARM_ROS_PATH/test/piper/test_move_l.yaml)" -1

# 圆弧运动（起点 → 中间点 → 终点）
ros2 topic pub /control/move_c geometry_msgs/msg/PoseArray \
  "$(cat AGX_ARM_ROS_PATH/test/piper/test_move_c.yaml)" -1

# 夹爪控制（宽度: 0.05m, 力: 1.0N）
ros2 topic pub /control/gripper agx_arm_msgs/msg/GripperCmd \
  "{width: 0.05, force: 1.0}" -1
```

### Nero 机械臂

```bash
# 关节运动
ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
  "$(cat AGX_ARM_ROS_PATH/test/nero/test_move_j.yaml)" -1

# 笛卡尔空间运动
ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped \
  "$(cat AGX_ARM_ROS_PATH/test/nero/test_move_p.yaml)" -1
```

### 服务调用

```bash
# 使能机械臂
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"

# 失能机械臂
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"

# 回零位
ros2 service call /move_home std_srvs/srv/Empty

# 退出示教模式（Piper 系列）
ros2 service call /exit_teach_mode std_srvs/srv/Empty
```

### 状态订阅

```bash
# 关节状态
ros2 topic echo /feedback/joint_states

# TCP 位姿
ros2 topic echo /feedback/tcp_pose

# 机械臂状态
ros2 topic echo /feedback/arm_status

# 夹爪状态
ros2 topic echo /feedback/gripper_status
```

---

## 注意事项

### CAN 通信

- 使用前**必须先激活 CAN 模块**
- 波特率：**1000000 bps**
- 若出现 `SendCanMessage failed` 错误，请检查 CAN 连接

### ⚠️ 安全警告

- **保持安全距离**：机械臂运动时，请勿进入其工作空间，以免造成伤害
- **奇异点风险**：靠近运动学奇异点时，关节可能发生突然大幅运动
- **MIT 模式危险**：高速响应的 MIT 模式极具危险性，请谨慎使用
