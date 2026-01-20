# AGX_Arm_Moveit2

[中文](README_ZH.md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

| PYTHON  | STATE |
|---------|-------|
| ![humble](https://img.shields.io/badge/ros-humble-blue.svg) | ![Pass](https://img.shields.io/badge/Pass-blue.svg) |


## 1 Install Moveit2

1) **Binary Installation** - [Reference link](https://moveit.ai/install-moveit2/binary/)

    ```bash
    sudo apt install ros-humble-moveit*
    ```

2) **Source Compilation** - [Reference link](https://moveit.ai/install-moveit2/source/)

## 2 Environment Setup

After installing Moveit2, install the necessary dependencies:

```bash
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
```

If your system locale is not set to English, configure it as follows:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

## 3 Moveit Control for Real Robot Arm

### 3.1 Start `agx_arm_ros`

After configuring [agx_arm_ros](../../README_EN.md#Getting-Started):

```bash
cd ~/agx_arm_ros
source install/setup.bash
bash can_activate.sh can0 1000000
```

Start the control node:

```bash
ros2 launch agx_arm start_single_agx_arm.launch.py gripper_val_mutiple:=2
```

### 3.2 Moveit2 Control

Start Moveit2:

```bash
cd ~/agx_arm_ros
conda deactivate  # Remove this line if Conda is not installed
source install/setup.bash
ros2 launch agx_arm_moveit demo.launch.py joint_states:=/control/joint_states
```

![agx_arm_moveit](../../asserts/pictures/piper_moveit.png)

You can control the robot arm by dragging the arrow at the end effector.

After adjusting the position, click **Plan & Execute** under the **Motion Planning** panel to start motion planning and execution.
