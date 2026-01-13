# AgileX Robotic Arm ROS2 Driver (Humble)

## Overview

The **AgileX Robotic Arm ROS2 Driver** package provides a complete ROS2 driver for AgileX robotic arms, supporting joint control, Cartesian space control, MoveIt integration, and various other functions.

---

## Getting Started

### Dependencies

#### System Dependencies

   ```bash
   # CAN tools
   sudo apt update && sudo apt install can-utils ethtool

   # ROS2 dependencies (for ROS2 driver)
   sudo apt install ros-$ROS_DISTRO-ros2-control
   sudo apt install ros-$ROS_DISTRO-ros2-controllers
   sudo apt install ros-$ROS_DISTRO-controller-manager
   sudo apt install ros-$ROS_DISTRO-topic-tools

   # Optional dependencies
   sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
   sudo apt install ros-$ROS_DISTRO-robot-state-publisher
   sudo apt install ros-$ROS_DISTRO-xacro
   ```

#### Python Dependencies

   ```bash
   pip3 install python-can>=4.3.1

   # For TracIK
   git clone https://github.com/chenhaox/pytracik.git
   cd pytracik
   pip3 install -r requirements.txt
   sudo apt install libboost-all-dev libeigen3-dev liborocos-kdl-dev libnlopt-dev libnlopt-cxx-dev
   python3 setup_linux.py install --user
   ```

### Installation

#### ROS2 Driver Installation

0. First, finish all steps in [Dependencies](#dependencies)

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
   cd ../../
   colcon build
   source install/setup.bash
   ```

#### Python SDK Installation

   ```bash
   # TODO
   pip3 install sdk
   ```

### Use CAN Module

Note: The CAN module here only supports the robot arm's built-in CAN module, and does not support other CAN modules.

Install CAN tools

   ```bash
   sudo apt update && sudo apt install can-utils ethtool
   ```

These two tools are used to configure the CAN module.

If executing the bash script shows `ip: command not found`, install the `ip` command by running `sudo apt-get install iproute2`

#### Find CAN Modules

Run the following command:

   ```bash
   bash find_all_can_port.sh
   ```

Enter the password, and if the CAN module is inserted into the computer and detected, it will output something like:

   ```bash
   Both ethtool and can-utils are installed.
   Interface can0 is connected to USB port 3-1.4:1.0
   ```

If there are multiple modules, the output will look like this:

   ```bash
   Both ethtool and can-utils are installed.
   Interface can0 is connected to USB port 3-1.4:1.0
   Interface can1 is connected to USB port 3-1.1:1.0
   ```

For each CAN module, there will be a corresponding output like `Interface can1 is connected to USB port 3-1.1:1.0`

Where `can1` is the name of the CAN module detected by the system, and `3-1.1:1.0` is the USB port it is connected to.

If the CAN module has already been activated with a different name, for example `can_arm`, the output will look like:

   ```bash
   Both ethtool and can-utils are installed.
   Interface can_arm is connected to USB port 3-1.4:1.0
   Interface can0 is connected to USB port 3-1.1:1.0
   ```

If no CAN module is detected, only the following will be output:

   ```bash
   Both ethtool and can-utils are installed.
   ```

#### Activate a Single CAN Module (use can_activate.sh script)

There are two situations for activating a single CAN module: one is when only one CAN module is connected to the PC, and the other is when multiple CAN modules are inserted, but only one is activated.

##### PC has only one USB-to-CAN module connected

Simply execute:

   ```bash
   bash can_activate.sh can0 1000000
   ```

Here, `can0` can be replaced with any name, and `1000000` is the baud rate, which cannot be changed.

##### PC has multiple USB-to-CAN modules connected, but only one module is activated at a time

Note: This case applies when using both the robot arm and the chassis.

(1) Find the USB hardware address of the CAN module. Unplug all CAN modules and plug in only the one connected to the robot arm, then execute:

   ```bash
   bash find_all_can_port.sh
   ```

Record the USB port value, for example, 3-1.4:1.0.

(2) Activate the CAN device. Assuming the USB port value is 3-1.4:1.0, run:

   ```bash
   bash can_activate.sh can_arm 1000000 "3-1.4:1.0"
   ```

Explanation: **3-1.4:1.0 is the hardware-encoded USB port, and the CAN device inserted there is renamed as can_arm, with a baud rate of 1000000 and activated.**

(3) Check if activation was successful by running `ifconfig` to see if `can_arm` appears. If it does, the CAN module is configured successfully.

#### Activate Multiple CAN Modules Simultaneously (use can_muti_activate.sh script)

First, determine how many official CAN modules are plugged into the PC (assumed here as 2).

Note: **If the current computer has 5 CAN modules inserted, you can only activate the specified CAN module**

##### Record the USB port hardware address of each CAN module

For each CAN module, unplug and reinsert it while recording the corresponding USB port address.

In the `can_muti_activate.sh` file, the `EXPECTED_CAN_COUNT` parameter should be set to the desired number of activated CAN modules (assumed here as 2).

(1) Plug in one CAN module and run:

   ```bash
   bash find_all_can_port.sh
   ```

Record the `USB port` value, for example, `3-1.4:1.0`

(2) Plug in the next CAN module. Ensure **it is not inserted into the same USB port as the previous one** and run:

   ```bash
   bash find_all_can_port.sh
   ```

Record the value of the second CAN module's `USB port`, for example `3-1.1:1.0`

Note: **If not previously activated, the first module will default to "can0," and the second will be "can1." If previously activated, the names will be the ones used before.**

##### Predefine USB ports, target interface names, and their bitrates

Assume the recorded `USB port` values are `3-1.4:1.0` and `3-1.1:1.0`. Replace the values inside the brackets in `USB_PORTS["1-9:1.0"]="can_left:1000000"` with `3-1.4:1.0` and `3-1.1:1.0`.

The final result is:

   ```bash
   USB_PORTS["3-1.4:1.0"]="can_left:1000000"
   USB_PORTS["3-1.1:1.0"]="can_right:1000000"
   ```

Explanation: **3-1.4:1.0 is the hardware-encoded USB port, the CAN device inserted there is renamed to "can_left," with a baud rate of 1000000, and activated.**

##### Activate multiple CAN modules

Run the script:

```bash
bash can_muti_activate.sh
```

##### Check if multiple CAN modules were set up successfully

Run `ifconfig` to check if `can_left` and `can_right` are present.

---

## ROS2 Driver Usage

### Launch Files

The ROS2 driver provides the following launch files:

#### AGX Arm Control Launch
```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py

# TODO
# Nero arm with test1 end effector, enable upper IK
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py arm_type:=nero end_effector_type:=test1 use_upper_ik:=true
```

#### RViz Control
```bash
# For Nero arm
ros2 launch agx_arm_description gazebo.launch.py arm_type:=nero 

# For Piper arm 
ros2 launch agx_arm_description gazebo.launch.py arm_type:=piper 
```

#### MoveIt Integration
```bash
# For Nero arm
ros2 launch nero_moveit demo.launch.py joint_states:=/control/joint_states

# For Piper arm (when available)
ros2 launch piper_moveit demo.launch.py joint_states:=/control/joint_states
```

## ROS2 Topics and Services

### Published Topics
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/control/joint_states` | `sensor_msgs/JointState` | Joint position control |
| `/control/end_pose` | `geometry_msgs/PoseStamped` | End-effector pose control |

### Subscribed Topics
| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/feedback/joint_states` | `sensor_msgs/JointState` | Joint state feedback |
| `/feedback/end_pose` | `geometry_msgs/PoseStamped` | End-effector pose feedback |

### Services
| Service | Type | Description |
|---------|------|-------------|
| `/enable_agx_arm` | `std_srvs/SetBool` | Enable/disable robot arm |
| `/move_home` | `std_srvs/Trigger` | Move to home position |

---

## Important Notes

### CAN Communication

- **Always activate CAN modules before use**
- **Use correct baud rate**: 1000000 bps
- **Check CAN connection** if messages show `SendCanMessage failed`

### CRITICAL SAFETY WARNINGS
- **Always maintain a safe distance from the moving robot arm during operation.** Entering the robot's workspace while it is active can result in severe injury.
- **Numerical solutions near kinematic singularities may cause sudden, large joint movements.** Maintain a safe distance as the arm's motion can become unpredictable.
- **The high-speed response MIT mode (Command 0xAD) is highly dangerous.** Use with extreme caution and only when absolutely necessary. Always maintain a safe distance from the robot when this mode is active.
