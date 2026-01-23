# AgileX 机械臂 ROS2 驱动 (Humble)

[English](./README_EN.md)

## 概述

**AgileX 机械臂 ROS2 驱动包** 为 AgileX 系列机械臂提供完整的 ROS2 驱动。

---

## 快速开始

### 依赖

   ```bash
   pip3 install python-can>=4.3.1 scipy numpy

   # CAN 工具
   sudo apt update && sudo apt install can-utils ethtool

   # ROS2 依赖
   sudo apt install ros-$ROS_DISTRO-ros2-control
   sudo apt install ros-$ROS_DISTRO-ros2-controllers
   sudo apt install ros-$ROS_DISTRO-controller-manager
   sudo apt install ros-$ROS_DISTRO-topic-tools

   # 可选依赖
   sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
   sudo apt install ros-$ROS_DISTRO-robot-state-publisher
   sudo apt install ros-$ROS_DISTRO-xacro
   ```

### 安装

#### ROS2 驱动安装

0. 首先完成 [依赖项](#依赖项) 中的所有步骤

1. 创建 ROS 工作空间并进入 src 目录：
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```

2. 克隆所需仓库：
   ```bash
   # TODO
   git clone <repository-url> nero
   ```

3. 构建工作空间：
   ```bash
   cd agx_arm_ros
   colcon build
   source install/setup.bash
   ```

#### Python SDK 安装

   ```bash
   # TODO
   pip3 install sdk
   ```
   
---

## ROS2 驱动使用

### 系统激活can模块
详见: [CAN_USER](./docs/CAN_USER.md)

### 启动文件

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py
```

### 启动参数

| 参数 | 默认值 | 描述 | 选项 |
|------|--------|------|------|
| `can_port` | `can0` | CAN 端口名称 | - |
| `arm_type` | `piper_h` | 机械臂类型 | `piper`, `piper_h`, `piper_l`, `piper_x`, `nero` |
| `effector_type` | `none` | 末端执行器类型 | `none`, `agx_gripper`, `revo2` |
| `auto_enable` | `True` | 启动时自动使能机械臂 | `True`, `False` |
| `installation_pos` | `horizontal` | 机械臂安装位置 | `horizontal`, `left`, `right` |
| `speed_percent` | `100` | 运动速度百分比 | `0-100` |
| `pub_rate` | `200` | 反馈发布频率 (Hz) | - |
| `enable_timeout` | `5.0` | 使能/失能超时时间 (秒) | - |
| `log_level` | `info` | 日志级别 | `debug`, `info`, `warn`, `error`, `fatal` |

---

## ROS2 话题和服务

### 反馈话题（发布）

| 话题 | 消息类型 | 描述 |
|------|----------|------|
| `/feedback/joint_states` | `sensor_msgs/JointState` | 当前关节位置 (rad) |
| `/feedback/end_pose` | `geometry_msgs/PoseStamped` | 当前末端位姿 |
| `/feedback/arm_status` | `agx_arm_msgs/AgxArmStatus` | 机械臂状态（控制模式、运动状态、错误等） |
| `/feedback/arm_ctrl_states` | `sensor_msgs/JointState` | 关节控制状态（仅 Piper） |
| `/feedback/gripper_status` | `agx_arm_msgs/GripperStatus` | 夹爪状态|
| `/feedback/hand_status` | `agx_arm_msgs/HandStatus` | 灵巧手状态|

### 控制话题（订阅）

| 话题 | 消息类型 | 描述 | 支持的机械臂 |
|------|----------|------|--------------|
| `/control/move_j` | `sensor_msgs/JointState` | 关节位置控制 | 全部 |
| `/control/move_p` | `geometry_msgs/PoseStamped` | 末端位姿控制 | 全部 |
| `/control/move_l` | `geometry_msgs/PoseStamped` | 直线运动控制 | 仅 Piper |
| `/control/move_c` | `agx_arm_msgs/TriplePose` | 圆弧运动控制 | 仅 Piper |
| `/control/move_js` | `sensor_msgs/JointState` | MIT 模式关节位置运动 | 仅 Piper |
| `/control/move_mit` | `sensor_msgs/JointState` | MIT 模式控制 | 仅 Piper |
| `/control/gripper` | `agx_arm_msgs/GripperCmd` | 夹爪控制 | 若已配置夹爪 |
| `/control/hand` | `agx_arm_msgs/HandCmd` | 灵巧手控制（位置/速度/电流） | 若已配置灵巧手 |
| `/control/hand_position_time` | `agx_arm_msgs/HandPositionTimeCmd` | 灵巧手位置+时间控制 | 如已配置灵巧手 |

### 服务

| 服务 | 类型 | 描述 |
|------|------|------|
| `/enable_agx_arm` | `std_srvs/SetBool` | 使能 (True) / 失能 (False) 机械臂 |
| `/move_home` | `std_srvs/Empty` | 移动所有关节到零位 |
| `/exit_teach_mode` | `std_srvs/Empty` | 退出示教模式并重置机械臂 |

---

## 重要注意事项

### CAN 通信

- **使用前必须先激活 CAN 模块**
- **使用正确的波特率**：1000000 bps
- **如果消息显示 `SendCanMessage failed`**，请检查 CAN 连接

### 严重安全警告

- **操作期间始终与移动的机械臂保持安全距离。** 在机械臂激活时进入其工作空间可能会造成严重伤害。
- **运动学奇异点附近的数值解可能导致关节突然大幅运动。** 机械臂的运动可能变得不可预测，请保持安全距离。
- **高速响应 MIT 模式极其危险。** 请谨慎使用，仅在绝对必要时使用。
