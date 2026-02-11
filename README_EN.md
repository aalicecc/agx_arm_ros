# AgileX Robotic Arm ROS2 Driver (Humble)

[中文](./README.md)

## Overview

This driver package provides full ROS2 interface support for AgileX series robotic arms (Piper, Nero, etc.).

---

## Quick Start

### 1. Install Dependencies

1. Python dependencies

    ```bash
    pip3 install "python-can>=4.3.1" scipy numpy
    ```

2. CAN tools

    ```bash
    sudo apt update && sudo apt install can-utils ethtool
    ```

3. ROS2 dependencies

    ```bash
    sudo apt install ros-$ROS_DISTRO-ros2-control \
                    ros-$ROS_DISTRO-ros2-controllers \
                    ros-$ROS_DISTRO-controller-manager \
                    ros-$ROS_DISTRO-topic-tools \
                    ros-$ROS_DISTRO-joint-state-publisher-gui \
                    ros-$ROS_DISTRO-robot-state-publisher \
                    ros-$ROS_DISTRO-xacro
    ```

### 2. Install Python SDK

```bash
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
pip3 install .
```

### 3. Install ROS2 Driver

Check if you are in a virtual environment. If so, it is recommended to exit the virtual environment first.

```bash
which pip3
```

1. Create workspace

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```

2. Clone repository

    ```bash
    git clone https://github.com/aalicecc/agx_arm_ros.git
    ```

3. Build

    ```bash
    cd ~/catkin_ws
    colcon build
    source install/setup.bash
    ```

---

## Usage

### Activate CAN Module

CAN module must be activated before use. See: [CAN Configuration Guide](./docs/CAN_USER.md)

### Launch Driver

You can start the driver using a launch file or by running the node directly.

> **Important: Read before launching**
> The parameters in the following launch commands **must** be replaced according to your actual hardware configuration:
> - **`can_port`**: The CAN port connected to the arm, e.g. `can0`.
> - **`arm_type`**: The arm model, e.g. `piper`.
> - **`effector_type`**: The end-effector type, e.g. `none` or `agx_gripper`.
> - **`tcp_offset`**: Tool Center Point (TCP) offset, e.g. [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] (Note: All values of this parameter must be floating-point numbers; for TCP offset configuration examples, see [TCP Offset Guide](./docs/tcp_offset/TCP_OFFSET_EN.md)).
>
> For full parameter descriptions, default values and options, see **[Launch Parameters](#launch-parameters)** below.


**Using launch file:**

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=piper effector_type:=none tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

**Running node directly:**

```bash
ros2 run agx_arm_ctrl agx_arm_ctrl_single --ros-args -p can_port:=can0 -p arm_type:=piper -p effector_type:=none -p tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
```

### Launch Parameters

| Parameter | Default | Description | Options |
|-----------|---------|-------------|---------|
| `can_port` | `can0` | CAN port | - |
| `arm_type` | `piper` | Arm model | `piper`, `piper_h`, `piper_l`, `piper_x`, `nero` |
| `effector_type` | `none` | End-effector type | `none`, `agx_gripper`, `revo2` |
| `auto_enable` | `True` | Auto enable on startup | `True`, `False` |
| `installation_pos` | `horizontal` | Mount orientation (Piper series only) | `horizontal`, `left`, `right` |
| `payload` | `empty` | Payload config (Piper series only for now) | `full`, `half`, `empty` |
| `speed_percent` | `100` | Motion speed (%) | `0-100` |
| `pub_rate` | `200` | Status publish rate (Hz) | - |
| `enable_timeout` | `5.0` | Enable timeout (seconds) | - |
| `tcp_offset` | `[0.0,0.0,0.0,0.0,0.0,0.0]` | TCP offset [x,y,z,rx,ry,rz] | - |
| `log_level` | `info` | Log level | `debug`, `info`, `warn`, `error`, `fatal` |

---

## Control Examples

Open an additional terminal and run the following commands:

```bash
cd ~/catkin_ws
source install/setup.bash
cd src/agx_arm_ros
```

### Piper Arm

1. Joint motion

    ```bash
    ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
      "$(cat test/piper/test_move_j.yaml)" -1
    ```

2. Point-to-point motion

    ```bash
    ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped \
      "$(cat test/piper/test_move_p.yaml)" -1
    ```

3. Linear motion

    ```bash
    ros2 topic pub /control/move_l geometry_msgs/msg/PoseStamped \
      "$(cat test/piper/test_move_l.yaml)" -1
    ```

4. Circular motion (start → middle → end)

    ```bash
    ros2 topic pub /control/move_c geometry_msgs/msg/PoseArray \
      "$(cat test/piper/test_move_c.yaml)" -1
    ```

### Nero Arm

1. Joint motion

    ```bash
    ros2 topic pub /control/move_j sensor_msgs/msg/JointState \
      "$(cat test/nero/test_move_j.yaml)" -1
    ```

2. Point-to-point motion

    ```bash
    ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped \
      "$(cat test/nero/test_move_p.yaml)" -1
    ```

### Gripper

1. Gripper control (via `/control/joint_states`)

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/gripper/test_gripper_joint_states.yaml)" -1
    ```

2. Arm + Gripper combined control (via `/control/joint_states`)

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/piper/test_arm_gripper_joint_states.yaml)" -1
    ```

### Dexterous Hand

1. Dexterous hand — Position mode (all fingers move to 10)

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_position.yaml)" -1
    ```

2. Dexterous hand — Speed mode (all fingers speed 50)

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_speed.yaml)" -1
    ```

3. Dexterous hand — Current mode (all fingers current 50)

    ```bash
    ros2 topic pub /control/hand agx_arm_msgs/msg/HandCmd \
      "$(cat test/hand/test_hand_current.yaml)" -1
    ```

4. Dexterous hand — Position-time control (all fingers move to 50, time 1 second)

    ```bash
    ros2 topic pub /control/hand_position_time agx_arm_msgs/msg/HandPositionTimeCmd \
      "$(cat test/hand/test_hand_position_time.yaml)" -1
    ```

5. Dexterous hand control (via `/control/joint_states`)

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/hand/test_hand_joint_states.yaml)" -1
    ```

6. Arm + Dexterous hand combined control (via `/control/joint_states`)

    ```bash
    ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
      "$(cat test/piper/test_arm_hand_joint_states.yaml)" -1
    ```

### Service Calls

1. Enable arm

    ```bash
    ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
    ```

2. Disable arm

    ```bash
    ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
    ```

3. Move to home position

    ```bash
    ros2 service call /move_home std_srvs/srv/Empty
    ```

4. Exit teach mode (Piper series)

    ```bash
    ros2 service call /exit_teach_mode std_srvs/srv/Empty
    ```

    > **⚠️ Important Safety Note:** 
    > 1. After executing this command, the robotic arm will first perform a homing operation and then restart automatically; there is a risk of falling during this process. It is recommended to gently hold the robotic arm after homing to prevent damage from falling.
    > 2. For Piper series robotic arms with firmware version 1.8.5 and above, the seamless mode switching feature is supported. There is no need to execute the above service command to exit teach mode, as the system will complete the mode switch automatically, avoiding the aforementioned fall risk.

### Status Subscription

1. Joint states

    ```bash
    ros2 topic echo /feedback/joint_states
    ```

2. TCP pose

    ```bash
    ros2 topic echo /feedback/tcp_pose
    ```

3. Arm status

    ```bash
    ros2 topic echo /feedback/arm_status
    ```

4. Master joint angles(For master arm mode)

    ```bash
    ros2 topic echo /feedback/master_joint_angles
    ```

5. Gripper status

    ```bash
    ros2 topic echo /feedback/gripper_status
    ```

6. Dexterous hand status 

    ```bash
    ros2 topic echo /feedback/hand_status
    ```

---

## ROS2 Interface

### Feedback Topics

| Topic | Message Type | Description | Condition |
|-------|--------------|-------------|-----------|
| `/feedback/joint_states` | `sensor_msgs/JointState` | Joint states | Always available |
| `/feedback/tcp_pose` | `geometry_msgs/PoseStamped` | TCP pose | Always available |
| `/feedback/arm_status` | `agx_arm_msgs/AgxArmStatus` | Arm status | Always available |
| `/feedback/master_joint_angles` | `sensor_msgs/JointState` | Master joint angles | Piper series |
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
| `effort` | Torque (N) |

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

| Topic                           | Message Type                       | Description           | Condition          |
| ----------------------------- | ---------------------------------- | ------------ | ------------- |
| `/control/joint_states`       | `sensor_msgs/JointState`           | Joint control (with end-effector) | Always available          |
| `/control/move_j`             | `sensor_msgs/JointState`           | Joint control motion       | Always available          |
| `/control/move_p`             | `geometry_msgs/PoseStamped`        | Point-to-point motion        | Always available          |
| `/control/move_l`             | `geometry_msgs/PoseStamped`        | Linear motion         | Piper series      |
| `/control/move_c`             | `geometry_msgs/PoseArray`          | Circular motion         | Piper series      |
| `/control/move_js`            | `sensor_msgs/JointState`           | MIT mode joint motion   | Piper series      |
| `/control/move_mit`           | `agx_arm_msgs/MoveMITMsg`          | MIT torque control     | Piper series      |
| `/control/hand`               | `agx_arm_msgs/HandCmd`             | Dexterous hand control        | Revo2 configured      |
| `/control/hand_position_time` | `agx_arm_msgs/HandPositionTimeCmd` | Hand position-time control    | Revo2 configured      |

#### `/control/joint_states` Details

This topic uses the `sensor_msgs/JointState` message type and supports simultaneous control of arm joints and end-effector (gripper/dexterous hand). Only the joints to be controlled need to be sent; joints not included will not be affected.

**Message Field Description:**

| Field | Description |
|-------|-------------|
| `name` | Joint name list |
| `position` | Target position for corresponding joints |
| `velocity` | Not used (can be left empty) |
| `effort` | Used for gripper force control (only effective for `gripper` joint) |

**Gripper control via `/control/joint_states`** (requires `effector_type=agx_gripper`)

Include `gripper` in `name`, set target width via `position`, and set gripping force via `effort`.

| Joint Name | position (width) | effort (force) |
|------------|-----------------|----------------|
| `gripper` | Target width (m), range: [0.0, 0.1] | Target force (N), range: [0.5, 3.0], default: 1.0 |

> **Note:** When `effort` is 0 or not specified, the default force of 1.0N is used.

Example: Control gripper width to 0.05m with force 1.5N
```bash
ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
  "{name: [gripper], position: [0.05], velocity: [], effort: [1.5]}" -1
```

**Dexterous hand control via `/control/joint_states`** (requires `effector_type=revo2`)

Include dexterous hand joint names in `name`, set target position via `position` (position mode, range [0, 100]). Only the joints to be controlled need to be sent; joints not included will maintain their current position.

Example: Control only the left index finger to position 80
```bash
ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
  "{name: [l_f_joint2], position: [80], velocity: [], effort: []}" -1
```

| Joint Name | Description | Position Range |
|------------|-------------|---------------|
| `l_f_joint1_1` / `r_f_joint1_1` | Thumb base | [0, 100] |
| `l_f_joint1_2` / `r_f_joint1_2` | Thumb tip | [0, 100] |
| `l_f_joint2` / `r_f_joint2` | Index finger | [0, 100] |
| `l_f_joint3` / `r_f_joint3` | Middle finger | [0, 100] |
| `l_f_joint4` / `r_f_joint4` | Ring finger | [0, 100] |
| `l_f_joint5` / `r_f_joint5` | Pinky finger | [0, 100] |


### Services

| Service | Type | Description | Condition |
|---------|------|-------------|-----------|
| `/enable_agx_arm` | `std_srvs/SetBool` | Enable/disable arm | Always available |
| `/move_home` | `std_srvs/Empty` | Move to home position | Always available |
| `/exit_teach_mode` | `std_srvs/Empty` | Exit teach mode | Piper series |

---

## Parameter Limits

### Gripper

| Parameter | Range | Default | Description |
|-----------|-------|---------|-------------|
| width | [0.0, 0.1] m | - | Target opening width |
| force | [0.5, 3.0] N | 1.0 | Gripping force |

> ⚠️ Values out of range will be rejected (not executed), and the node will output a warning log. For example: when sending force=5.0, the command will not be executed, and a warning `force must be in range [0.5, 3.0], current value: 5.0` will be output.

### Dexterous Hand (Revo2)

| Parameter | Range | Description |
|-----------|-------|-------------|
| position | [0, 100] | Finger target position, 0 = fully open, 100 = fully closed |
| speed | [-100, 100] | Finger motion speed |
| current | [-100, 100] | Finger drive current |
| time | [0, 255] | Time to reach target position (unit: 10ms, e.g. 100 = 1 second) |

> ⚠️ Values out of range will be rejected (not executed), and the node will output a warning log. For example: when sending position=120, the command will not be executed, and a warning `position must be in range [0, 100], current value: 120` will be output.

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
