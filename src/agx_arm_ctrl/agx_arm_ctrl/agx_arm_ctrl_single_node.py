#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
import rclpy
import math
import threading
import numpy as np
from typing import Optional
from enum import IntEnum
from pyAgxArm import create_agx_arm_config, AgxArmFactory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from std_srvs.srv import SetBool, Empty
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R

from agx_arm_msgs.msg import (
    AgxArmStatus, GripperStatus, GripperCmd, 
    HandStatus, HandCmd, HandPositionTimeCmd, TriplePose
)
from agx_arm_ctrl.effector import AgxGripperWrapper, Revo2Wrapper

class ControlMode(IntEnum):
    TEACH = 0x02

class AgxArmRosNode(Node):

    def __init__(self):
        super().__init__("agx_arm_ctrl_single_node")

        ### ros parameters
        self._declare_parameters()
        self._load_parameters()
        self._log_parameters()

        ### AgxArmFactory
        self._init_agx_arm()

        ### effector
        self._init_effector()
    
        ### variables
        self.enable_flag = False

        ### publisher
        self._setup_publishers()

        ### subscribers
        self._setup_subscribers()

        ### services
        self._setup_services()

        ### publisher thread
        self.publisher_thread = threading.Thread(target=self._publish_thread)
        self.publisher_thread.start()

    ### initialization methods

    def _declare_parameters(self):
        self.declare_parameter("can_port", "can0")
        self.declare_parameter("pub_rate", 200)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("arm_type", "piper")
        self.declare_parameter("speed_percent", 100)
        self.declare_parameter("enable_timeout", 5.0)
        self.declare_parameter("installation_pos", "horizontal")
        self.declare_parameter("effector_type", "none")

    def _load_parameters(self):
        self.can_port = self.get_parameter("can_port").value
        self.pub_rate = self.get_parameter("pub_rate").value
        self.auto_enable = self.get_parameter("auto_enable").value
        self.arm_type = self.get_parameter("arm_type").value
        self.speed_percent = self.get_parameter("speed_percent").value
        self.enable_timeout = self.get_parameter("enable_timeout").value
        self.installation_pos = self.get_parameter("installation_pos").value
        self.effector_type = self.get_parameter("effector_type").value

    def _log_parameters(self):
        self.get_logger().info(f"can_port: {self.can_port}")
        self.get_logger().info(f"pub_rate: {self.pub_rate}")
        self.get_logger().info(f"auto_enable: {self.auto_enable}")
        self.get_logger().info(f"arm_type: {self.arm_type}")
        self.get_logger().info(f"speed_percent: {self.speed_percent}")
        self.get_logger().info(f"enable_timeout: {self.enable_timeout}")
        self.get_logger().info(f"installation_pos: {self.installation_pos}")
        self.get_logger().info(f"effector_type: {self.effector_type}")

    def _init_agx_arm(self):
        config = create_agx_arm_config(
            robot=self.arm_type, comm="can", channel=self.can_port
        )
        self.agx_arm = AgxArmFactory.create_arm(config)
        self.agx_arm.connect()
        self.agx_arm.set_speed_percent(self.speed_percent)
        self.arm_joint_names = list(config["joint_limit"].keys())
        self.arm_joint_count = len(self.arm_joint_names)
        self.is_piper = "piper" in self.arm_type
        # TODO :
        if self.is_piper:
            self.agx_arm.set_installation_pos(self.installation_pos)

    def _init_effector(self):
        self.gripper: Optional[AgxGripperWrapper] = None
        self.hand: Optional[Revo2Wrapper] = None

        if self.effector_type == "agx_gripper":
            self.gripper = AgxGripperWrapper(self.agx_arm)
            if self.gripper.initialize():
                self.get_logger().info("AgxGripper initialized successfully")
            else:
                self.get_logger().error("Failed to initialize AgxGripper")
                self.gripper = None
        elif self.effector_type == "revo2":
            self.hand = Revo2Wrapper(self.agx_arm)
            if self.hand.initialize():
                self.get_logger().info("Revo2 hand initialized successfully")
            else:
                self.get_logger().error("Failed to initialize Revo2 hand")
                self.hand = None

    def _setup_publishers(self):
        self.joint_states_pub = self.create_publisher(
            JointState, "/feedback/joint_states", 1
        )
        self.end_pose_pub = self.create_publisher(
            PoseStamped, "/feedback/end_pose", 1
        )
        self.arm_status_pub = self.create_publisher(
            AgxArmStatus, "/feedback/arm_status", 1
        )
        # TODO :arm_ctrl_states
        if self.gripper is not None:
            self.gripper_status_pub = self.create_publisher(
                GripperStatus, "/feedback/gripper_status", 1
            )
            self.arm_ctrl_states_pub = self.create_publisher(
            JointState, "/feedback/arm_ctrl_states", 1
            )   
        if self.hand is not None:
            self.hand_status_pub = self.create_publisher(
                HandStatus, "/feedback/hand_status", 1
            )

    def _setup_subscribers(self):
        self.create_subscription(
            JointState, "/control/move_j", self._move_j_callback, 1
        )
        self.create_subscription(
            PoseStamped, "/control/move_p", self._move_p_callback, 1
        )
        if self.is_piper:
            self.create_subscription(
                PoseStamped, "/control/move_l", self._move_l_callback, 1
            )
            self.create_subscription(
                TriplePose, "/control/move_c", self._move_c_callback, 1
            )
            self.create_subscription(
                JointState, "/control/move_mit", self._move_mit_callback, 1
            )
            self.create_subscription(
                JointState, "/control/move_js", self._move_js_callback, 1
            )
        if self.gripper is not None:
            self.create_subscription(
                GripperCmd, "/control/gripper", self._gripper_cmd_callback, 1
            )
        if self.hand is not None:
            self.create_subscription(
                HandCmd, "/control/hand", self._hand_cmd_callback, 1
            )
            self.create_subscription(
                HandPositionTimeCmd, "/control/hand_position_time", 
                self._hand_position_time_cmd_callback, 1
            )

    def _setup_services(self):
        self.create_service(SetBool, "/enable_agx_arm", self._enable_callback)
        self.create_service(Empty, "/move_home", self._move_home_callback)
        self.create_service(Empty, "/exit_teach_mode", self._exit_teach_mode_callback)

    ### utility methods

    def _float_to_ros_time(self, timestamp: float) -> Time:
        """将float时间戳转换为ROS Time消息"""
        ros_time = Time()
        ros_time.sec = int(timestamp)
        ros_time.nanosec = int((timestamp - ros_time.sec) * 1e9)
        return ros_time

    def _safe_get_value(self, array, index, default=0.0):
        if index >= len(array):
            return default
        value = array[index]
        return default if math.isnan(value) else value

    def _check_arm_ready(self) -> bool:
        joint_states = self.agx_arm.get_joint_states()
        if joint_states is None or joint_states.hz <= 0:
            return False
        return True

    def _check_can_control(self) -> bool:
        if not self._check_arm_ready():
            self.get_logger().warn("Agx_arm is not connected, cannot control")
            return False
        if not self.enable_flag:
            self.get_logger().warn("Agx_arm is not enabled, cannot control")
            return False
        arm_status = self.agx_arm.get_arm_status()
        if arm_status is not None and arm_status.msg.ctrl_mode == ControlMode.TEACH:
            self.get_logger().warn("Agx_arm is in teach mode, cannot control")
            return False
        return True

    def _create_pose_cmd(self, pose: Pose) -> list:
        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        pose_xyz = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ]
        euler_angles = R.from_quat(quaternion).as_euler("xyz", degrees=False)
        return pose_xyz + euler_angles.tolist()

    def _enable_arm(self, enable: bool = True, timeout: float = 5.0) -> bool:
        start_time = time.time()
        action_name = "enable" if enable else "disable"
        
        while not (self.agx_arm.enable() if enable else self.agx_arm.disable()):
            if time.time() - start_time > timeout:
                self.get_logger().error(
                    f"Timeout waiting for arm to {action_name} after {timeout} seconds"
                )
                return False
            time.sleep(0.01)
        
        joints_status = self.agx_arm.get_joints_enable_status_list()
        all_joints_in_target_status = all(joints_status) if enable else not all(joints_status) 
        
        if all_joints_in_target_status:
            self.enable_flag = True if enable else False
            self.get_logger().info(f"All joints {action_name} status is {self.enable_flag}")
        else:
            self.get_logger().warn(
                f"Not all joints are {action_name}d after {action_name}ing the arm"
            )
        
        return True

    ### publisher thread

    def _publish_thread(self):
        rate = self.create_rate(self.pub_rate)

        if rclpy.ok() and self.auto_enable:
            if not self._enable_arm(True, self.enable_timeout):
                self.get_logger().error("Failed to auto-enable the arm")

        # publishing loop
        while rclpy.ok():
            if self.agx_arm.is_ok():
                self._publish_joint_states()
                self._publish_end_pose()
                self._publish_arm_status()
                self._publish_effector_status()
                # TODO :
                if self.is_piper :
                    self._publish_arm_ctrl_states()
            rate.sleep()
    
    ### publish methods

    def _publish_joint_states(self):
        joint_states = self.agx_arm.get_joint_states()
        if joint_states is None or joint_states.hz <= 0:
            return

        msg = JointState()
        msg.header.stamp = self._float_to_ros_time(joint_states.timestamp)
        msg.name = self.arm_joint_names
        msg.position = joint_states.msg
        msg.velocity = [0.0] * self.arm_joint_count
        msg.effort = [0.0] * self.arm_joint_count
        self.joint_states_pub.publish(msg)

    def _publish_end_pose(self):
        end_pose = self.agx_arm.get_ee_pose()
        if end_pose is None or end_pose.hz <= 0:
            return

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = end_pose.msg[0:3]
        roll, pitch, yaw = end_pose.msg[3:6]
        quaternion = R.from_euler("xyz", [roll, pitch, yaw]).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion

        msg = PoseStamped()
        msg.header.stamp = self._float_to_ros_time(end_pose.timestamp)
        msg.pose = pose
        self.end_pose_pub.publish(msg)

    def _publish_arm_status(self):
        arm_status = self.agx_arm.get_arm_status()
        if arm_status is None:
            return

        msg = AgxArmStatus()
        msg.ctrl_mode = arm_status.msg.ctrl_mode
        msg.arm_status = arm_status.msg.arm_status
        msg.mode_feedback = arm_status.msg.mode_feedback
        msg.teach_status = arm_status.msg.teach_status
        msg.motion_status = arm_status.msg.motion_status
        msg.trajectory_num = arm_status.msg.trajectory_num
        err = arm_status.msg.err_status
        for i in range(self.arm_joint_count):
            angle_limit = getattr(err, f"joint_{i+1}_angle_limit")
            comm_status = getattr(err, f"communication_status_joint_{i+1}")

            msg.joint_angle_limit.append(angle_limit)
            msg.communication_status_joint.append(comm_status)

        self.arm_status_pub.publish(msg)

    def _publish_arm_ctrl_states(self):
        arm_ctrl_states = self.agx_arm.get_joint_ctrl_states()
        if arm_ctrl_states is None:
            return

        msg = JointState()
        msg.header.stamp = self._float_to_ros_time(arm_ctrl_states.timestamp)
        msg.name = self.arm_joint_names
        msg.position = arm_ctrl_states.msg
        msg.velocity = [0.0] * self.arm_joint_count
        msg.effort = [0.0] * self.arm_joint_count
        self.arm_ctrl_states_pub.publish(msg)

    def _publish_gripper_status(self):
        status = self.gripper.get_status()
        if status is not None:
            msg = GripperStatus()
            msg.header.stamp = self._float_to_ros_time(status.timestamp)
            msg.width = status.width
            msg.force = status.force
            msg.voltage_too_low = status.voltage_too_low
            msg.motor_overheating = status.motor_overheating
            msg.driver_overcurrent = status.driver_overcurrent
            msg.driver_overheating = status.driver_overheating
            msg.sensor_status = status.sensor_status
            msg.driver_error_status = status.driver_error_status
            msg.driver_enable_status = status.driver_enable_status
            msg.homing_status = status.homing_status
            self.gripper_status_pub.publish(msg)

    def _publish_hand_status(self):
        hand_status = self.hand.get_status()
        finger_pos = self.hand.get_finger_position()
        if hand_status is not None:
            msg = HandStatus()
            msg.header.stamp = self._float_to_ros_time(hand_status.timestamp)
            msg.left_or_right = hand_status.left_or_right
            # status
            msg.thumb_tip_status = hand_status.thumb_tip
            msg.thumb_base_status = hand_status.thumb_base
            msg.index_finger_status = hand_status.index_finger
            msg.middle_finger_status = hand_status.middle_finger
            msg.ring_finger_status = hand_status.ring_finger
            msg.pinky_finger_status = hand_status.pinky_finger
            # position
            if finger_pos is not None:
                msg.thumb_tip_pos = finger_pos.thumb_tip
                msg.thumb_base_pos = finger_pos.thumb_base
                msg.index_finger_pos = finger_pos.index_finger
                msg.middle_finger_pos = finger_pos.middle_finger
                msg.ring_finger_pos = finger_pos.ring_finger
                msg.pinky_finger_pos = finger_pos.pinky_finger
            self.hand_status_pub.publish(msg)

    def _publish_effector_status(self):
        if self.gripper is not None and self.gripper.is_ok():
            self._publish_gripper_status()
        if self.hand is not None and self.hand.is_ok():
            self._publish_hand_status()

    ### arm control callbacks

    def _move_j_callback(self, msg: JointState):
        if not self._check_can_control():
            return

        joint_pos = {}
        for idx, joint_name in enumerate(msg.name):
            joint_pos[joint_name] = self._safe_get_value(msg.position, idx)
        joints = [joint_pos.get(f'j{i}', 0) for i in range(1, self.arm_joint_count + 1)]
        self.agx_arm.move_j(joints)

    def _move_p_callback(self, msg: PoseStamped):
        if not self._check_can_control():
            return

        pose_cmd = self._create_pose_cmd(msg.pose)
        self.agx_arm.move_p(pose_cmd)

    def _move_l_callback(self, msg: PoseStamped):
        if not self.is_piper:
            self.get_logger().warn("move_l just piper series supported")
            return
        if not self._check_can_control():
            return

        pose_cmd = self._create_pose_cmd(msg.pose)
        self.agx_arm.move_l(pose_cmd)

    def _move_c_callback(self, msg: TriplePose):
        if not self.is_piper:
            self.get_logger().warn("move_c just piper series supported")
            return
        if not self._check_can_control():
            return

        pose_start = self._create_pose_cmd(msg.start_pose.pose)
        pose_mid = self._create_pose_cmd(msg.mid_pose.pose)
        pose_end = self._create_pose_cmd(msg.end_pose.pose)
        self.agx_arm.move_c(pose_start, pose_mid, pose_end)

    def _move_js_callback(self, msg: JointState):
        if not self.is_piper:
            self.get_logger().warn("move_js just piper series supported")
            return
        if not self._check_can_control():
            return

        joint_pos = {}
        for idx, joint_name in enumerate(msg.name):
            joint_pos[joint_name] = self._safe_get_value(msg.position, idx)
        joints = [joint_pos.get(f'j{i}', 0) for i in range(1, self.arm_joint_count + 1)]
        self.agx_arm.move_js(joints)

    def _move_mit_callback(self, msg: JointState):
        if not self.is_piper:
            self.get_logger().warn("move_mit just piper series supported")
            return
        if not self._check_can_control():
            return

        joint_pos = {}          
        for idx, joint_name in enumerate(msg.name):
            joint_pos[joint_name] = self._safe_get_value(msg.position, idx)
        for i in range(1, self.arm_joint_count + 1):
            self.agx_arm.move_mit(joint_index=i, p_des=joint_pos.get(f'j{i}', 0), v_des=0, kp=10.0, kd=0.8, t_ff=0)

    ### effector control callbacks

    def _gripper_cmd_callback(self, msg: GripperCmd):
        if self.gripper is None:
            self.get_logger().warn("gripper not initialized")
            return

        try:
            self.gripper.move(width=msg.width, force=msg.force)
        except ValueError as e:
            self.get_logger().error(f"gripper control param error: {e}")

    def _hand_position_time_cmd_callback(self, msg: HandPositionTimeCmd):
        if self.hand is None:
            self.get_logger().warn("revo2 hand not initialized")
            return
        
        try:
            self.hand.position_time_ctrl(
                mode="pos",
                thumb_tip=msg.thumb_tip_pos,
                thumb_base=msg.thumb_base_pos,
                index_finger=msg.index_finger_pos,
                middle_finger=msg.middle_finger_pos,
                ring_finger=msg.ring_finger_pos,
                pinky_finger=msg.pinky_finger_pos,
            )
            time.sleep(0.02)
            self.hand.position_time_ctrl(
                mode="time",
                thumb_tip=msg.thumb_tip_time,
                thumb_base=msg.thumb_base_time,
                index_finger=msg.index_finger_time,
                middle_finger=msg.middle_finger_time,
                ring_finger=msg.ring_finger_time,
                pinky_finger=msg.pinky_finger_time,
            )
        except ValueError as e:
            self.get_logger().error(f"hand control param error: {e}")

    def _hand_cmd_callback(self, msg: HandCmd):
        if self.hand is None:
            self.get_logger().warn("revo2 hand not initialized")
            return
        
        mode_to_method = {
            "position": self.hand.position_ctrl,
            "speed": self.hand.speed_ctrl,
            "current": self.hand.current_ctrl,
        }

        mode = msg.mode.lower()        
        if mode not in mode_to_method:
            self.get_logger().warn(f"unknown hand control mode: {mode}")
            return

        try:
            mode_to_method[mode](
                thumb_tip=msg.thumb_tip,
                thumb_base=msg.thumb_base,
                index_finger=msg.index_finger,
                middle_finger=msg.middle_finger,
                ring_finger=msg.ring_finger,
                pinky_finger=msg.pinky_finger,
            )
            # TODO :test hand
            time.sleep(0.05)
            print(f"fps",self.hand.get_fps())
            print(f"pos",self.hand.get_finger_position())
            print(f"speed",self.hand.get_finger_speed())
            print(f"current",self.hand.get_finger_current())

        except ValueError as e:
            self.get_logger().error(f"hand control param error: {e}")

    ### service callbacks

    def _enable_callback(self, request, response):
        try:
            if not self._check_arm_ready():
                response.success = False
                self.get_logger().warn("Agx_arm is not connected, cannot set enable state")
            elif request.data:
                response.success = True if self._enable_arm(True) else False
            else:
                response.success = True if self._enable_arm(False) else False
            
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Failed to set enable state: {str(e)}")
        return response

    def _move_home_callback(self, request, response):
        try:
            if not self._check_arm_ready():
                self.get_logger().warn("Agx_arm is not connected, cannot move to home position")
            elif not self.enable_flag:
                self.get_logger().warn("Agx_arm is not enabled, cannot move to home position")
            else:
                arm_status = self.agx_arm.get_arm_status()
                if arm_status is not None and arm_status.msg.ctrl_mode == ControlMode.TEACH:
                    self.get_logger().warn("Agx_arm is in teach mode, cannot move to home position")
                else:
                    self.agx_arm.move_j([0] * self.arm_joint_count)
                    self.get_logger().info("Agx_arm moved to home position successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to move to home position: {str(e)}")
        return response

    def _exit_teach_mode_callback(self, request, response):
        try:
            arm_status = self.agx_arm.get_arm_status()
            if arm_status is not None and arm_status.msg.ctrl_mode == ControlMode.TEACH:
                if self.is_piper:
                    self.agx_arm.move_js([0] * self.arm_joint_count)
                    time.sleep(2)
                self.agx_arm.electronic_emergency_stop()
                time.sleep(0.3)
                self.agx_arm.reset()
                self.get_logger().info("Exited teach mode successfully")
            else:
                self.get_logger().info("Agx_arm is not in teach mode")
        except Exception as e:
            self.get_logger().error(f"Failed to exit teach mode: {e}")
        return response


def main(args=None):
    rclpy.init(args=args)

    try:
        node = AgxArmRosNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
