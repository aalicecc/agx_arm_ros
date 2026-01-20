# AgileX 机械臂 ROS2 驱动 (Humble)

## 概述

**AgileX 机械臂 ROS2 驱动包** 为 AgileX 系列机械臂提供完整的 ROS2 驱动，支持关节控制、笛卡尔空间控制、MoveIt 集成等多种功能。

---

## 快速开始

### 依赖项

#### 系统依赖

   ```bash
   # CAN 工具
   sudo apt update && sudo apt install can-utils ethtool

   # ROS2 依赖（用于 ROS2 驱动）
   sudo apt install ros-$ROS_DISTRO-ros2-control
   sudo apt install ros-$ROS_DISTRO-ros2-controllers
   sudo apt install ros-$ROS_DISTRO-controller-manager
   sudo apt install ros-$ROS_DISTRO-topic-tools

   # 可选依赖
   sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
   sudo apt install ros-$ROS_DISTRO-robot-state-publisher
   sudo apt install ros-$ROS_DISTRO-xacro
   ```

#### Python 依赖

   ```bash
   pip3 install python-can>=4.3.1

   # TracIK 相关依赖
   git clone https://github.com/chenhaox/pytracik.git
   cd pytracik
   pip3 install -r requirements.txt
   sudo apt install libboost-all-dev libeigen3-dev liborocos-kdl-dev libnlopt-dev libnlopt-cxx-dev
   python3 setup_linux.py install --user
   ```

### 安装

#### ROS2 驱动安装

0. 首先完成 [依赖项](#依赖项) 中的所有步骤

1. 创建 ROS 工作空间并导航到 src 目录：
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
   cd ../../
   colcon build
   source install/setup.bash
   ```

#### Python SDK 安装

   ```bash
   # TODO
   pip3 install sdk
   ```

### 使用 CAN 模块

注意：此 CAN 模块仅支持机械臂内置 CAN 模块，不支持其他 CAN 模块。

安装 CAN 工具

   ```bash
   sudo apt update && sudo apt install can-utils ethtool
   ```

这两个工具用于配置 CAN 模块。

如果执行 bash 脚本显示 `ip: command not found`，请运行 `sudo apt-get install iproute2` 安装 ip 命令

#### 查找 CAN 模块

运行以下命令：

   ```bash
   bash find_all_can_port.sh
   ```

输入密码，如果 CAN 模块已插入电脑并被检测到，将输出类似信息：

   ```bash
   Both ethtool and can-utils are installed.
   Interface can0 is connected to USB port 3-1.4:1.0
   ```

如果有多个模块，输出将如下：

   ```bash
   Both ethtool and can-utils are installed.
   Interface can0 is connected to USB port 3-1.4:1.0
   Interface can1 is connected to USB port 3-1.1:1.0
   ```

每个 CAN 模块都会有类似 `Interface can1 is connected to USB port 3-1.1:1.0` 的输出。

其中 `can1` 是系统检测到的 CAN 模块名称，`3-1.1:1.0` 是其连接的 USB 端口。

如果 CAN 模块已被重命名为其他名称，例如 `can_arm`，输出将如下：

   ```bash
   Both ethtool and can-utils are installed.
   Interface can_arm is connected to USB port 3-1.4:1.0
   Interface can0 is connected to USB port 3-1.1:1.0
   ```

如果未检测到 CAN 模块，仅输出：

   ```bash
   Both ethtool and can-utils are installed.
   ```

#### 激活单个 CAN 模块（使用 can_activate.sh 脚本）

激活单个 CAN 模块有两种情况：一种是 PC 上只连接了一个 CAN 模块，另一种是多个 CAN 模块插入但一次只激活一个。

##### PC 上只连接了一个 USB 转 CAN 模块

只需执行：

   ```bash
   bash can_activate.sh can0 1000000
   ```

此处 `can0` 可以替换为任意名称，`1000000` 是波特率，不能更改。

##### PC 上连接了多个 USB 转 CAN 模块，但一次只激活一个模块

注意：此情况适用于同时使用机械臂和底盘的情况。

(1) 查找 CAN 模块的 USB 硬件地址。先拔掉所有 CAN 模块，插入连接机械臂的那个，然后执行：

   ```bash
   bash find_all_can_port.sh
   ```

记录 USB 端口值，例如 3-1.4:1.0。

(2) 激活 CAN 设备。假设 USB 端口值为 3-1.4:1.0，运行：

   ```bash
   bash can_activate.sh can_arm 1000000 "3-1.4:1.0"
   ```

说明：**3-1.4:1.0 是硬件编码的 USB 端口，插入该处的 CAN 设备被重命名为 can_arm，波特率为 1000000，并激活。**

(3) 运行 `ifconfig` 检查激活是否成功。如果出现 `can_arm`，则 CAN 模块配置成功。

#### 同时激活多个 CAN 模块（使用 can_muti_activate.sh 脚本）

首先确定插入 PC 的官方 CAN 模块数量（假设为 2 个）。

注意：**如果当前电脑插入了 5 个 CAN 模块，您只能激活指定的 CAN 模块**

##### 记录每个 CAN 模块的 USB 端口硬件地址

对每个 CAN 模块，拔插并记录相应的 USB 端口地址。

在 `can_muti_activate.sh` 文件中，将 `EXPECTED_CAN_COUNT` 参数设置为期望激活的 CAN 模块数量（假设为 2）。

(1) 插入一个 CAN 模块并运行：

   ```bash
   bash find_all_can_port.sh
   ```

记录 `USB port` 值，例如 `3-1.4:1.0`

(2) 插入下一个 CAN 模块。确保 **没有插入与上一个相同的 USB 端口** 并运行：

   ```bash
   bash find_all_can_port.sh
   ```

记录第二个 CAN 模块的 `USB port` 值，例如 `3-1.1:1.0`

注意：**如果之前未激活，第一个模块默认为"can0"，第二个为"can1"。如果之前已激活，则名称为之前使用的名称。**

##### 预定义 USB 端口、目标接口名称和波特率

假设记录的 `USB port` 值为 `3-1.4:1.0` 和 `3-1.1:1.0`。将 `USB_PORTS["1-9:1.0"]="can_left:1000000"` 中括号内的值替换为 `3-1.4:1.0` 和 `3-1.1:1.0`。

最终结果为：

   ```bash
   USB_PORTS["3-1.4:1.0"]="can_left:1000000"
   USB_PORTS["3-1.1:1.0"]="can_right:1000000"
   ```

说明：**3-1.4:1.0 是硬件编码的 USB 端口，插入该处的 CAN 设备被重命名为 "can_left"，波特率为 1000000，并激活。**

##### 激活多个 CAN 模块

运行脚本：

```bash
bash can_muti_activate.sh
```

##### 检查多个 CAN 模块是否设置成功

运行 `ifconfig` 检查是否出现 `can_left` 和 `can_right`。

---

## ROS2 驱动使用

### 启动文件

ROS2 驱动提供以下启动文件：

#### AGX 机械臂控制启动
```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py

# TODO
# Nero 机械臂，test1 末端，启用上层 IK
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py arm_type:=nero end_effector_type:=test1 use_upper_ik:=true
```

#### RViz 控制
```bash
# Nero 机械臂
ros2 launch nero_description gazebo.launch.py 

# Piper 机械臂
ros2 launch piper_description gazebo.launch.py 
```

#### MoveIt 集成
```bash
# Nero 机械臂
ros2 launch nero_moveit demo.launch.py joint_states:=/control/joint_states

# Piper 机械臂
ros2 launch piper_moveit demo.launch.py joint_states:=/control/joint_states
```

---

## ROS2 话题和服务

### 发布话题
| 话题 | 消息类型 | 描述 |
|-------|---------|------|
| `/control/joint_states` | `sensor_msgs/JointState` | 关节位置控制 |
| `/control/end_pose` | `geometry_msgs/PoseStamped` | 末端姿态控制 |

### 订阅话题
| 话题 | 消息类型 | 描述 |
|-------|---------|------|
| `/feedback/joint_states` | `sensor_msgs/JointState` | 关节状态反馈 |
| `/feedback/end_pose` | `geometry_msgs/PoseStamped` | 末端姿态反馈 |

### 服务
| 服务 | 类型 | 描述 |
|------|------|------|
| `/enable_agx_arm` | `std_srvs/SetBool` | 使能/禁用机械臂 |
| `/move_home` | `std_srvs/Trigger` | 移动到 HOME 位置 |

---

## 重要注意事项

### CAN 通信

- **使用前必须先激活 CAN 模块**
- **使用正确的波特率**：1000000 bps
- **如果消息显示 `SendCanMessage failed`**，请检查 CAN 连接

### 严重安全警告
- **操作期间始终与移动的机械臂保持安全距离。** 在机械臂激活时进入其工作空间可能会造成严重伤害。
- **运动学奇异点附近的数值解可能导致关节突然大幅运动。** 机械臂的运动可能变得不可预测，请保持安全距离。
- **高速响应 MIT 模式（命令 0xAD）极其危险。** 请谨慎使用，仅在绝对必要时使用。在此模式激活时，请始终与机械臂保持安全距离。
