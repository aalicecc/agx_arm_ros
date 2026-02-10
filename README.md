# AgileX 机械臂 ROS2 驱动 (Humble)

[English](./README_EN.md)

## 概述

本驱动包为 AgileX 系列机械臂（Piper、Nero 等）提供完整的 ROS2 接口支持。

---

## 快速开始

### 1. 安装依赖

```bash
# Python 依赖
pip3 install "python-can>=4.3.1" scipy numpy

# CAN 工具
sudo apt update && sudo apt install can-utils ethtool

# ROS2 依赖
sudo apt install ros-$ROS_DISTRO-ros2-control \
                 ros-$ROS_DISTRO-ros2-controllers \
                 ros-$ROS_DISTRO-controller-manager \
                 ros-$ROS_DISTRO-topic-tools \
                 ros-$ROS_DISTRO-joint-state-publisher-gui \
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

检查是否在虚拟环境里，如果是，建议先退出虚拟环境。

```bash
which pip3
```

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

您可以通过 launch 文件或直接运行节点来启动驱动。

> **重要提示：启动前必读**
> 以下启动命令中的参数**必须**根据您的实际硬件配置进行替换：
> - **`can_port`**：机械臂连接的 CAN 端口，示例值 `can0`。
> - **`arm_type`**：机械臂的型号，示例值 `piper`。
> - **`effector_type`**：末端执行器类型，示例值 `none` 或 `agx_gripper`。
> - **`tcp_offset`**：工具中心点（TCP）偏移量，示例值：[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]（注：该参数所有值均需为浮点数；关于 TCP 偏移实际配置示例，请参考 [TCP 设置详解](./docs/tcp_offset/TCP_OFFSET.md)）。
>
> 所有参数的完整说明、默认值及可选值，请参阅下方的 **[启动参数](#启动参数)** 。


**使用 launch 文件启动：**

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=none tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

**直接运行节点启动:**

```bash
ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args -p can_port:=can0 -p arm_type:=piper -p effector_type:=none -p tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

### 启动参数

| 参数 | 默认值 | 说明 | 可选值 |
|------|--------|------|--------|
| `can_port` | `can0` | CAN 端口 | - |
| `arm_type` | `piper` | 机械臂型号 | `piper`, `piper_h`, `piper_l`, `piper_x`, `nero` |
| `effector_type` | `none` | 末端执行器类型 | `none`, `agx_gripper`, `revo2` |
| `auto_enable` | `True` | 启动时自动使能 | `True`, `False` |
| `installation_pos` | `horizontal` | 安装方向 (Piper 系列专用) | `horizontal`, `left`, `right` |
| `payload` | `empty` | 负载配置 (暂时只对piper系列生效) | `full`, `half`, `empty` |
| `speed_percent` | `100` | 运动速度 (%) | `0-100` |
| `pub_rate` | `200` | 状态发布频率 (Hz) | - |
| `enable_timeout` | `5.0` | 使能超时 (秒) | - |
| `tcp_offset` | `[0.0,0.0,0.0,0.0,0.0,0.0]` | TCP 偏移 [x,y,z,rx,ry,rz] | - |
| `log_level` | `info` | 日志级别 | `debug`, `info`, `warn`, `error`, `fatal` |

---

## 控制示例

额外启动一个终端,运行以下指令:

```bash
cd ~/catkin_ws
source install/setup.bash
cd src/agx_arm_ros
```

### Piper 机械臂

1. 关节运动

    ```bash
    ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
      "$(cat test/piper/test_move_j.yaml)" -1
    ```

2. 点到点运动

    ```bash
    ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped \
      "$(cat test/piper/test_move_p.yaml)" -1
    ```

3. 直线运动

    ```bash
    ros2 topic pub /control/move_l geometry_msgs/msg/PoseStamped \
      "$(cat test/piper/test_move_l.yaml)" -1
    ```

4. 圆弧运动（起点 → 中间点 → 终点）

    ```bash
    ros2 topic pub /control/move_c geometry_msgs/msg/PoseArray \
      "$(cat test/piper/test_move_c.yaml)" -1
    ```

### Nero 机械臂

1. 关节运动

    ```bash
    ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
      "$(cat test/nero/test_move_j.yaml)" -1
    ```

2. 点到点运动

    ```bash
    ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped \
      "$(cat test/nero/test_move_p.yaml)" -1
    ```

### Gripper 夹爪

1. 夹爪控制（通过 `/control/joint_states`控制）

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/gripper/test_gripper_joint_states.yaml)" -1
    ```

2. 机械臂 + 夹爪联合控制（通过 `/control/joint_states`控制）

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/piper/test_arm_gripper_joint_states.yaml)" -1
    ```

### Hand 灵巧手

1. 灵巧手 — 位置模式（所有手指移动到 10）

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_position.yaml)" -1
    ```

2. 灵巧手 — 速度模式（所有手指速度 50）

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_speed.yaml)" -1
    ```

3. 灵巧手 — 电流模式（所有手指电流 50）

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_current.yaml)" -1
    ```

4. 灵巧手 — 位置-时间控制（所有手指移动到 50，时间 1 秒）

    ```bash
    ros2 topic pub /control/hand_position_time agx_arm_msgs/msg/HandPositionTimeCmd \
      "$(cat test/hand/test_hand_position_time.yaml)" -1
    ```

5. 灵巧手控制（通过 `/control/joint_states`控制）

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/hand/test_hand_joint_states.yaml)" -1
    ```

6. 机械臂 + 灵巧手联合控制（通过 `/control/joint_states`控制）

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/piper/test_arm_hand_joint_states.yaml)" -1
    ```

### 服务调用

1. 使能机械臂

    ```bash
    ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
    ```

2. 失能机械臂

    ```bash
    ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
    ```

3. 回零位

    ```bash
    ros2 service call /move_home std_srvs/srv/Empty
    ```

4. 退出示教模式（Piper 系列）

    ```bash
    ros2 service call /exit_teach_mode std_srvs/srv/Empty
    ```

    > **注意：** Piper 系列机器人若固件版本为 1.8.5 及以上，已支持 **模式无缝切换** 功能，无需执行上述退出示教模式的服务指令，系统会自动完成模式切换。

### 状态订阅

1. 关节状态

    ```bash
    ros2 topic echo /feedback/joint_states
    ```

2. TCP 位姿

    ```bash
    ros2 topic echo /feedback/tcp_pose
    ```

3. 机械臂状态

    ```bash
    ros2 topic echo /feedback/arm_status
    ```

4. 夹爪状态

    ```bash
    ros2 topic echo /feedback/gripper_status
    ```

---

## ROS2 接口

### 反馈话题

| 话题 | 消息类型 | 说明 | 适用条件 |
|------|----------|------|----------|
| `/feedback/joint_states` | `sensor_msgs/JointState` | 关节状态 | 始终可用 |
| `/feedback/tcp_pose` | `geometry_msgs/PoseStamped` | TCP 位姿 | 始终可用 |
| `/feedback/arm_status` | `agx_arm_msgs/AgxArmStatus` | 机械臂状态 | 始终可用 |
| `/feedback/master_joint_angles` | `sensor_msgs/JointState` | 主臂关节角度 | Piper 系列 |
| `/feedback/gripper_status` | `agx_arm_msgs/GripperStatus` | 夹爪状态 | 配置 AgxGripper |
| `/feedback/hand_status` | `agx_arm_msgs/HandStatus` | 灵巧手状态 | 配置 Revo2 |

#### `/feedback/joint_states` 详细说明

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
| `effort` | 力矩（N） |

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

| 话题                            | 消息类型                               | 说明           | 适用条件          |
| ----------------------------- | ---------------------------------- | ------------ | ------------- |
| `/control/joint_states`       | `sensor_msgs/JointState`           | 关节控制（含末端执行器） | 始终可用          |
| `/control/move_j`             | `sensor_msgs/JointState`           | 关节控制运动       | 始终可用          |
| `/control/move_p`             | `geometry_msgs/PoseStamped`        | 点到点运动        | 始终可用          |
| `/control/move_l`             | `geometry_msgs/PoseStamped`        | 直线运动         | Piper 系列      |
| `/control/move_c`             | `geometry_msgs/PoseArray`          | 圆弧运动         | Piper 系列      |
| `/control/move_js`            | `sensor_msgs/JointState`           | MIT 模式关节运动   | Piper 系列      |
| `/control/move_mit`           | `agx_arm_msgs/MoveMITMsg`          | MIT 力矩控制     | Piper 系列      |
| `/control/hand`               | `agx_arm_msgs/HandCmd`             | 灵巧手控制        | 配置 Revo2      |
| `/control/hand_position_time` | `agx_arm_msgs/HandPositionTimeCmd` | 灵巧手位置时间控制    | 配置 Revo2      |

#### `/control/joint_states` 详细说明

该话题使用 `sensor_msgs/JointState` 消息类型，支持同时控制机械臂关节和末端执行器（夹爪/灵巧手）。只需发送要控制的关节即可，未包含的关节不受影响。

**消息字段说明：**

| 字段 | 说明 |
|------|------|
| `name` | 关节名称列表 |
| `position` | 对应关节的目标位置 |
| `velocity` | 未使用（可留空） |
| `effort` | 用于夹爪力控制（仅对 `gripper` 关节有效） |

**通过 `/control/joint_states` 控制夹爪**（需配置 `effector_type=agx_gripper`）

在 `name` 中包含 `gripper`，通过 `position` 设置目标宽度，通过 `effort` 设置夹持力。

| 关节名 | position（宽度） | effort（力） |
|--------|-----------------|-------------|
| `gripper` | 目标宽度 (m)，范围: [0.0, 0.1] | 目标力 (N)，范围: [0.5, 3.0]，默认: 1.0 |

> **注意：** 当 `effort` 为 0 或未指定时，使用默认力 1.0N。

示例：控制夹爪宽度 0.05m、力 1.5N
```bash
ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
  "{name: [gripper], position: [0.05], velocity: [], effort: [1.5]}" -1
```

**通过 `/control/joint_states` 控制灵巧手**（需配置 `effector_type=revo2`）

在 `name` 中包含灵巧手关节名，通过 `position` 设置目标位置（position 模式，范围 [0, 100]）。仅需发送要控制的关节，未包含的关节将保持当前位置。

示例：仅控制左手食指到位置 80
```bash
ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
  "{name: [l_f_joint2], position: [80], velocity: [], effort: []}" -1
```

| 关节名 | 说明 | position 范围 |
|--------|------|--------------|
| `l_f_joint1_1` / `r_f_joint1_1` | 大拇指指根 | [0, 100] |
| `l_f_joint1_2` / `r_f_joint1_2` | 大拇指指尖 | [0, 100] |
| `l_f_joint2` / `r_f_joint2` | 食指 | [0, 100] |
| `l_f_joint3` / `r_f_joint3` | 中指 | [0, 100] |
| `l_f_joint4` / `r_f_joint4` | 无名指 | [0, 100] |
| `l_f_joint5` / `r_f_joint5` | 小指 | [0, 100] |


### 服务

| 服务 | 类型 | 说明 | 适用条件 |
|------|------|------|----------|
| `/enable_agx_arm` | `std_srvs/SetBool` | 使能/失能机械臂 | 始终可用 |
| `/move_home` | `std_srvs/Empty` | 回零位 | 始终可用 |
| `/exit_teach_mode` | `std_srvs/Empty` | 退出示教模式 | Piper 系列 |

---

## 参数限制

### 夹爪 (Gripper)

| 参数 | 范围 | 默认值 | 说明 |
|------|------|--------|------|
| width (宽度) | [0.0, 0.1] m | - | 目标开合宽度 |
| force (力) | [0.5, 3.0] N | 1.0 | 夹持力度 |

> ⚠️ 超出范围的值将被拒绝（不执行），节点输出警告日志。例如：发送 force=5.0 时，该指令不会执行，并输出 `force must be in range [0.5, 3.0], current value: 5.0` 警告。

### 灵巧手 (Revo2)

| 参数 | 范围 | 说明 |
|------|------|------|
| position (位置) | [0, 100] | 手指目标位置，0 为完全张开，100 为完全握紧 |
| speed (速度) | [-100, 100] | 手指运动速度 |
| current (电流) | [-100, 100] | 手指驱动电流 |
| time (时间) | [0, 255] | 到达目标位置的时间（单位: 10ms，例如 100 = 1 秒） |

> ⚠️ 超出范围的值将被拒绝（不执行），节点输出警告日志。例如：发送 position=120 时，该指令不会执行，并输出 `position must be in range [0, 100], current value: 120` 警告。

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
