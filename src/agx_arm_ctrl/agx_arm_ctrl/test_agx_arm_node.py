#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool, Empty
from scipy.spatial.transform import Rotation as R

from agx_arm_msgs.msg import (
    AgxArmStatus, TriplePose,
    GripperStatus, GripperCmd,
    HandStatus, HandCmd, HandPositionTimeCmd
)


class AgxArmTestNode(Node):
    def __init__(self):
        super().__init__("agx_arm_test_node")

        # ==================== 反馈话题订阅 ====================
        self.joint_states_sub = self.create_subscription(
            JointState, "/feedback/joint_states", self._joint_states_callback, 1
        )
        self.end_pose_sub = self.create_subscription(
            PoseStamped, "/feedback/end_pose", self._end_pose_callback, 1
        )
        self.arm_status_sub = self.create_subscription(
            AgxArmStatus, "/feedback/arm_status", self._arm_status_callback, 1
        )
        self.arm_ctrl_states_sub = self.create_subscription(
            JointState, "/feedback/arm_ctrl_states", self._arm_ctrl_states_callback, 1
        )
        self.gripper_status_sub = self.create_subscription(
            GripperStatus, "/feedback/gripper_status", self._gripper_status_callback, 1
        )
        self.hand_status_sub = self.create_subscription(
            HandStatus, "/feedback/hand_status", self._hand_status_callback, 1
        )

        # ==================== 控制话题发布 ====================
        # 机械臂控制
        self.move_j_pub = self.create_publisher(JointState, "/control/move_j", 1)
        self.move_p_pub = self.create_publisher(PoseStamped, "/control/move_p", 1)
        self.move_l_pub = self.create_publisher(PoseStamped, "/control/move_l", 1)
        self.move_c_pub = self.create_publisher(TriplePose, "/control/move_c", 1)
        self.move_js_pub = self.create_publisher(JointState, "/control/move_js", 1)
        self.move_mit_pub = self.create_publisher(JointState, "/control/move_mit", 1)
        
        # 末端执行器控制
        self.gripper_cmd_pub = self.create_publisher(GripperCmd, "/control/gripper", 1)
        self.hand_cmd_pub = self.create_publisher(HandCmd, "/control/hand", 1)
        self.hand_position_time_pub = self.create_publisher(
            HandPositionTimeCmd, "/control/hand_position_time", 1
        )

        # ==================== 服务客户端 ====================
        self.enable_client = self.create_client(SetBool, "/enable_agx_arm")
        self.move_home_client = self.create_client(Empty, "/move_home")
        self.exit_teach_client = self.create_client(Empty, "/exit_teach_mode")

        # ==================== 状态变量 ====================
        self.last_joint_states = None
        self.last_end_pose = None
        self.last_arm_status = None
        self.last_arm_ctrl_states = None
        self.last_gripper_status = None
        self.last_hand_status = None
        
        # 消息计数
        self.joint_states_count = 0
        self.end_pose_count = 0
        self.arm_status_count = 0
        self.arm_ctrl_states_count = 0
        self.gripper_status_count = 0
        self.hand_status_count = 0

        self._print_welcome()

    def _print_welcome(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("AGX Arm Test Node Initialized")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Available test categories:")
        self.get_logger().info("  [1-4]   Feedback topics")
        self.get_logger().info("  [5-7]   Services (enable/home/teach)")
        self.get_logger().info("  [8-13]  Arm control (move_j/p/l/c/js/mit)")
        self.get_logger().info("  [14-17] Gripper control")
        self.get_logger().info("  [18-22] Hand control")
        self.get_logger().info("  [99]    Run all tests")
        self.get_logger().info("=" * 60)

    # ==================== 回调函数 ====================

    def _joint_states_callback(self, msg: JointState):
        self.last_joint_states = msg
        self.joint_states_count += 1

    def _end_pose_callback(self, msg: PoseStamped):
        self.last_end_pose = msg
        self.end_pose_count += 1

    def _arm_status_callback(self, msg: AgxArmStatus):
        self.last_arm_status = msg
        self.arm_status_count += 1

    def _arm_ctrl_states_callback(self, msg: JointState):
        self.last_arm_ctrl_states = msg
        self.arm_ctrl_states_count += 1

    def _gripper_status_callback(self, msg: GripperStatus):
        self.last_gripper_status = msg
        self.gripper_status_count += 1

    def _hand_status_callback(self, msg: HandStatus):
        self.last_hand_status = msg
        self.hand_status_count += 1

    # ==================== 工具方法 ====================

    def _reset_counts(self):
        self.joint_states_count = 0
        self.end_pose_count = 0
        self.arm_status_count = 0
        self.arm_ctrl_states_count = 0
        self.gripper_status_count = 0
        self.hand_status_count = 0

    def _spin_for(self, duration: float):
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _get_joint_count(self) -> int:
        if self.last_joint_states is not None:
            return len(self.last_joint_states.name)
        return 6  # default

    def _get_joint_names(self) -> list:
        if self.last_joint_states is not None:
            return list(self.last_joint_states.name)
        return [f"j{i}" for i in range(1, 7)]

    def _log_section(self, title: str):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  {title}")
        self.get_logger().info("=" * 60)

    # ==================== 反馈话题测试 ====================

    def test_all_feedback(self, duration: float = 2.0) -> dict:
        """测试所有反馈话题"""
        self._log_section("Testing All Feedback Topics")
        
        self._reset_counts()
        self._spin_for(duration)
        
        results = {
            "joint_states": self.joint_states_count,
            "end_pose": self.end_pose_count,
            "arm_status": self.arm_status_count,
            "arm_ctrl_states": self.arm_ctrl_states_count,
            "gripper_status": self.gripper_status_count,
            "hand_status": self.hand_status_count,
        }
        
        self.get_logger().info(f"Message counts in {duration}s:")
        for topic, count in results.items():
            hz = count / duration
            status = "OK" if count > 0 else "NO DATA"
            self.get_logger().info(f"  {topic}: {count} msgs ({hz:.1f} Hz) [{status}]")
        
        return results

    def test_joint_states(self, duration: float = 2.0) -> bool:
        """测试关节状态话题"""
        self._log_section("Testing /feedback/joint_states")
        
        self._reset_counts()
        self._spin_for(duration)
        
        if self.last_joint_states is not None:
            self.get_logger().info(f"Received {self.joint_states_count} messages")
            self.get_logger().info(f"Joint names: {list(self.last_joint_states.name)}")
            positions = [f"{p:.4f}" for p in self.last_joint_states.position]
            self.get_logger().info(f"Positions (rad): {positions}")
            return True
        else:
            self.get_logger().warn("No joint states received!")
            return False

    def test_end_pose(self, duration: float = 2.0) -> bool:
        """测试末端位姿话题"""
        self._log_section("Testing /feedback/end_pose")
        
        self._reset_counts()
        self._spin_for(duration)
        
        if self.last_end_pose is not None:
            p = self.last_end_pose.pose.position
            o = self.last_end_pose.pose.orientation
            self.get_logger().info(f"Received {self.end_pose_count} messages")
            self.get_logger().info(f"Position: x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}")
            self.get_logger().info(f"Orientation: x={o.x:.4f}, y={o.y:.4f}, z={o.z:.4f}, w={o.w:.4f}")
            return True
        else:
            self.get_logger().warn("No end pose received!")
            return False

    def test_arm_status(self, duration: float = 2.0) -> bool:
        """测试机械臂状态话题"""
        self._log_section("Testing /feedback/arm_status")
        
        self._reset_counts()
        self._spin_for(duration)
        
        if self.last_arm_status is not None:
            s = self.last_arm_status
            self.get_logger().info(f"Received {self.arm_status_count} messages")
            self.get_logger().info(f"ctrl_mode: {s.ctrl_mode}, arm_status: {s.arm_status}")
            self.get_logger().info(f"mode_feedback: {s.mode_feedback}, teach_status: {s.teach_status}")
            self.get_logger().info(f"motion_status: {s.motion_status}, trajectory_num: {s.trajectory_num}")
            return True
        else:
            self.get_logger().warn("No arm status received!")
            return False

    def test_arm_ctrl_states(self, duration: float = 2.0) -> bool:
        """测试机械臂控制状态话题"""
        self._log_section("Testing /feedback/arm_ctrl_states")
        
        self._reset_counts()
        self._spin_for(duration)
        
        if self.last_arm_ctrl_states is not None:
            self.get_logger().info(f"Received {self.arm_ctrl_states_count} messages")
            self.get_logger().info(f"Joint names: {list(self.last_arm_ctrl_states.name)}")
            positions = [f"{p:.4f}" for p in self.last_arm_ctrl_states.position]
            self.get_logger().info(f"Ctrl positions (rad): {positions}")
            return True
        else:
            self.get_logger().warn("No arm ctrl states received!")
            return False

    # ==================== 服务测试 ====================

    def test_enable(self, enable: bool = True, timeout: float = 10.0) -> bool:
        """测试使能/失能服务"""
        action = "Enable" if enable else "Disable"
        self._log_section(f"Testing /enable_agx_arm ({action})")
        
        if not self.enable_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Enable service not available!")
            return False
        
        request = SetBool.Request()
        request.data = enable
        
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f"{action} result: success={result.success}")
            return result.success
        else:
            self.get_logger().error(f"{action} service call failed!")
            return False

    def test_move_home(self, timeout: float = 10.0) -> bool:
        """测试回零服务"""
        self._log_section("Testing /move_home")
        
        if not self.move_home_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Move home service not available!")
            return False
        
        request = Empty.Request()
        future = self.move_home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        self.get_logger().info("Move home command sent")
        self.get_logger().info("Waiting for motion to complete...")
        time.sleep(3.0)
        return True

    def test_exit_teach_mode(self, timeout: float = 10.0) -> bool:
        """测试退出示教模式服务"""
        self._log_section("Testing /exit_teach_mode")
        
        if not self.exit_teach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Exit teach mode service not available!")
            return False
        
        request = Empty.Request()
        future = self.exit_teach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        self.get_logger().info("Exit teach mode command sent")
        return True

    # ==================== 机械臂控制测试 ====================

    def test_move_j(self, joint_angles: list = None) -> bool:
        """测试move_j关节控制"""
        self._log_section("Testing /control/move_j")
        
        self._spin_for(0.5)  # 获取当前状态
        
        joint_count = self._get_joint_count()
        joint_names = self._get_joint_names()
        
        if joint_angles is None:
            joint_angles = [0.0, 0.4, -0.4, 0.0, -0.4, 0.0][:joint_count]
            # TODO:
            # joint_angles = [0.3, 0.8, 0.45, -1.5, 1.570796326794896619, 0.0][:joint_count]
            if joint_count > 6:
                joint_angles.extend([0.0] * (joint_count - 6))
        
        msg = JointState()
        msg.name = joint_names
        msg.position = joint_angles
        
        self.get_logger().info(f"Joint names: {joint_names}")
        self.get_logger().info(f"Target angles: {joint_angles}")
        self.move_j_pub.publish(msg)
        
        time.sleep(2.0)
        self._spin_for(0.5)
        
        if self.last_joint_states is not None:
            current = list(self.last_joint_states.position)
            errors = [abs(c - t) for c, t in zip(current, joint_angles)]
            self.get_logger().info(f"Max position error: {max(errors):.4f} rad")
        
        return True

    def test_move_p(self, pose: list = None) -> bool:
        """测试move_p末端控制"""
        self._log_section("Testing /control/move_p")
        
        self._spin_for(0.5)
        
        if self.last_end_pose is None:
            self.get_logger().error("Cannot get current end pose!")
            return False
        
        current = self.last_end_pose.pose
        
        if pose is None:
            # 在当前位置基础上小幅移动
            target_pos = [
                current.position.x,
                current.position.y,
                current.position.z + 0.02,  # 向上移动2cm
            ]
            quat = [
                current.orientation.x,
                current.orientation.y,
                current.orientation.z,
                current.orientation.w,
            ]
        else:
            target_pos = pose[:3]
            quat = R.from_euler("xyz", pose[3:6]).as_quat()
        
        msg = PoseStamped()
        msg.pose.position.x = target_pos[0]
        msg.pose.position.y = target_pos[1]
        msg.pose.position.z = target_pos[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.get_logger().info(f"Target position: {target_pos}")
        self.move_p_pub.publish(msg)
        
        time.sleep(2.0)
        return True

    def test_move_l(self, pose: list = None) -> bool:
        """测试move_l直线运动 (piper only)"""
        self._log_section("Testing /control/move_l (Piper only)")
        
        self._spin_for(0.5)
        
        if self.last_end_pose is None:
            self.get_logger().error("Cannot get current end pose!")
            return False
        
        current = self.last_end_pose.pose
        
        if pose is None:
            target_pos = [
                current.position.x + 0.02,  # 向前移动2cm
                current.position.y,
                current.position.z,
            ]
            quat = [
                current.orientation.x,
                current.orientation.y,
                current.orientation.z,
                current.orientation.w,
            ]
        else:
            target_pos = pose[:3]
            quat = R.from_euler("xyz", pose[3:6]).as_quat()
        
        msg = PoseStamped()
        msg.pose.position.x = target_pos[0]
        msg.pose.position.y = target_pos[1]
        msg.pose.position.z = target_pos[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.get_logger().info(f"Target position (linear): {target_pos}")
        self.move_l_pub.publish(msg)
        
        time.sleep(2.0)
        return True

    def test_move_c(self) -> bool:
        """测试move_c圆弧运动 (piper only)"""
        self._log_section("Testing /control/move_c (Piper only)")
        
        self._spin_for(0.5)
        
        if self.last_end_pose is None:
            self.get_logger().error("Cannot get current end pose!")
            return False
        
        current = self.last_end_pose.pose
        
        # 创建三个点形成圆弧
        start_pose = PoseStamped()
        start_pose.pose.position.x = 0.2
        start_pose.pose.position.y = 0.0
        start_pose.pose.position.z = 0.3
        start_pose.pose.orientation = current.orientation
        
        mid_pose = PoseStamped()
        mid_pose.pose.position.x = 0.2
        mid_pose.pose.position.y = 0.05
        mid_pose.pose.position.z = 0.35
        mid_pose.pose.orientation = current.orientation
        
        end_pose = PoseStamped()
        end_pose.pose.position.x = 0.2
        end_pose.pose.position.y = 0.0
        end_pose.pose.position.z = 0.4
        end_pose.pose.orientation = current.orientation
        
        msg = TriplePose()
        msg.start_pose = start_pose
        msg.mid_pose = mid_pose
        msg.end_pose = end_pose
        
        self.get_logger().info("Sending circular motion command")
        self.get_logger().info(f"  Start: ({start_pose.pose.position.x}, {start_pose.pose.position.y}, {start_pose.pose.position.z})")
        self.get_logger().info(f"  Mid:   ({mid_pose.pose.position.x}, {mid_pose.pose.position.y}, {mid_pose.pose.position.z})")
        self.get_logger().info(f"  End:   ({end_pose.pose.position.x}, {end_pose.pose.position.y}, {end_pose.pose.position.z})")
        self.move_c_pub.publish(msg)
        
        time.sleep(3.0)
        return True

    def test_move_js(self, joint_angles: list = None) -> bool:
        """测试move_js关节空间运动 (piper only)"""
        self._log_section("Testing /control/move_js (Piper only)")
        
        self._spin_for(0.5)
        
        joint_count = self._get_joint_count()
        joint_names = self._get_joint_names()
        
        if joint_angles is None:
            joint_angles = [0.0, 0.3, -0.3, 0.0, -0.3, 0.0][:joint_count]
            if joint_count > 6:
                joint_angles.extend([0.0] * (joint_count - 6))
        
        msg = JointState()
        msg.name = joint_names
        msg.position = joint_angles
        
        self.get_logger().info(f"Target angles (js): {joint_angles}")
        self.move_js_pub.publish(msg)
        
        time.sleep(2.0)
        return True

    def test_move_mit(self, joint_angles: list = None) -> bool:
        """测试MIT控制 (piper only)"""
        self._log_section("Testing /control/move_mit (Piper only)")
        
        self._spin_for(0.5)
        
        joint_count = self._get_joint_count()
        joint_names = self._get_joint_names()
        
        if joint_angles is None:
            joint_angles = [0.0, 0.2, -0.2, 0.0, -0.2, 0.0][:joint_count]
            if joint_count > 6:
                joint_angles.extend([0.0] * (joint_count - 6))
        
        msg = JointState()
        msg.name = joint_names
        msg.position = joint_angles
        
        self.get_logger().info(f"MIT target angles: {joint_angles}")
        self.move_mit_pub.publish(msg)
        
        time.sleep(2.0)
        return True

    # ==================== 夹爪控制测试 ====================

    def test_gripper_status(self, duration: float = 2.0) -> bool:
        """测试夹爪状态反馈"""
        self._log_section("Testing /feedback/gripper_status")
        
        self._reset_counts()
        self._spin_for(duration)
        
        if self.last_gripper_status is not None:
            s = self.last_gripper_status
            self.get_logger().info(f"Received {self.gripper_status_count} messages")
            self.get_logger().info(f"Width: {s.width:.4f} m, Force: {s.force:.2f} N")
            self.get_logger().info(f"Driver status:")
            self.get_logger().info(f"  voltage_too_low: {s.voltage_too_low}")
            self.get_logger().info(f"  motor_overheating: {s.motor_overheating}")
            self.get_logger().info(f"  driver_overcurrent: {s.driver_overcurrent}")
            self.get_logger().info(f"  driver_overheating: {s.driver_overheating}")
            self.get_logger().info(f"  sensor_status: {s.sensor_status}")
            self.get_logger().info(f"  driver_error_status: {s.driver_error_status}")
            self.get_logger().info(f"  driver_enable_status: {s.driver_enable_status}")
            self.get_logger().info(f"  homing_status: {s.homing_status}")
            return True
        else:
            self.get_logger().warn("No gripper status received (gripper may not be configured)")
            return False

    def test_gripper_open(self, width: float = 0.08, force: float = 1.0) -> bool:
        """测试夹爪打开"""
        self._log_section("Testing Gripper Open")
        
        msg = GripperCmd()
        msg.width = width
        msg.force = force
        
        self.get_logger().info(f"Opening gripper to width={width}m, force={force}N")
        self.gripper_cmd_pub.publish(msg)
        
        time.sleep(1.0)
        return True

    def test_gripper_close(self, force: float = 1.5) -> bool:
        """测试夹爪关闭"""
        self._log_section("Testing Gripper Close")
        
        msg = GripperCmd()
        msg.width = 0.0
        msg.force = force
        
        self.get_logger().info(f"Closing gripper with force={force}N")
        self.gripper_cmd_pub.publish(msg)
        
        time.sleep(1.0)
        return True

    def test_gripper_move(self, width: float = 0.05, force: float = 1.0) -> bool:
        """测试夹爪移动到指定宽度"""
        self._log_section("Testing Gripper Move")
        
        msg = GripperCmd()
        msg.width = width
        msg.force = force
        
        self.get_logger().info(f"Moving gripper to width={width}m, force={force}N")
        self.gripper_cmd_pub.publish(msg)
        
        time.sleep(1.0)
        return True

    # ==================== 灵巧手控制测试 ====================

    def test_hand_status(self, duration: float = 2.0) -> bool:
        """测试灵巧手状态反馈"""
        self._log_section("Testing /feedback/hand_status")
        
        self._reset_counts()
        self._spin_for(duration)
        
        if self.last_hand_status is not None:
            s = self.last_hand_status
            self.get_logger().info(f"Received {self.hand_status_count} messages")
            self.get_logger().info(f"Hand type: {'Left' if s.left_or_right == 1 else 'Right'}")
            self.get_logger().info(f"Motor status (0=idle, 1=running, 2=blocked):")
            self.get_logger().info(f"  thumb_tip: {s.thumb_tip_status}, thumb_base: {s.thumb_base_status}")
            self.get_logger().info(f"  index: {s.index_finger_status}, middle: {s.middle_finger_status}")
            self.get_logger().info(f"  ring: {s.ring_finger_status}, pinky: {s.pinky_finger_status}")
            self.get_logger().info(f"Finger positions [0-100]:")
            self.get_logger().info(f"  thumb_tip: {s.thumb_tip_pos}, thumb_base: {s.thumb_base_pos}")
            self.get_logger().info(f"  index: {s.index_finger_pos}, middle: {s.middle_finger_pos}")
            self.get_logger().info(f"  ring: {s.ring_finger_pos}, pinky: {s.pinky_finger_pos}")
            return True
        else:
            self.get_logger().warn("No hand status received (hand may not be configured)")
            return False

    def test_hand_position(self, positions: dict = None) -> bool:
        """测试灵巧手位置控制"""
        self._log_section("Testing Hand Position Control")
        
        if positions is None:
            positions = {
                "thumb_tip": 50,
                "thumb_base": 50,
                "index_finger": 50,
                "middle_finger": 50,
                "ring_finger": 50,
                "pinky_finger": 50,
            }
        
        msg = HandCmd()
        msg.mode = "position"
        msg.thumb_tip = positions.get("thumb_tip", 0)
        msg.thumb_base = positions.get("thumb_base", 0)
        msg.index_finger = positions.get("index_finger", 0)
        msg.middle_finger = positions.get("middle_finger", 0)
        msg.ring_finger = positions.get("ring_finger", 0)
        msg.pinky_finger = positions.get("pinky_finger", 0)
        
        self.get_logger().info(f"Position control: {positions}")
        self.hand_cmd_pub.publish(msg)
        
        time.sleep(1.0)
        return True

    def test_hand_speed(self, speeds: dict = None) -> bool:
        """测试灵巧手速度控制"""
        self._log_section("Testing Hand Speed Control")
        
        if speeds is None:
            speeds = {
                "thumb_tip": 30,
                "thumb_base": 30,
                "index_finger": 30,
                "middle_finger": 30,
                "ring_finger": 30,
                "pinky_finger": 30,
            }
        
        msg = HandCmd()
        msg.mode = "speed"
        msg.thumb_tip = speeds.get("thumb_tip", 0)
        msg.thumb_base = speeds.get("thumb_base", 0)
        msg.index_finger = speeds.get("index_finger", 0)
        msg.middle_finger = speeds.get("middle_finger", 0)
        msg.ring_finger = speeds.get("ring_finger", 0)
        msg.pinky_finger = speeds.get("pinky_finger", 0)
        
        self.get_logger().info(f"Speed control: {speeds}")
        self.hand_cmd_pub.publish(msg)
        
        time.sleep(1.0)
        return True

    def test_hand_current(self, currents: dict = None) -> bool:
        """测试灵巧手电流控制"""
        self._log_section("Testing Hand Current Control")
        
        if currents is None:
            currents = {
                "thumb_tip": 20,
                "thumb_base": 20,
                "index_finger": 20,
                "middle_finger": 20,
                "ring_finger": 20,
                "pinky_finger": 20,
            }
        
        msg = HandCmd()
        msg.mode = "current"
        msg.thumb_tip = currents.get("thumb_tip", 0)
        msg.thumb_base = currents.get("thumb_base", 0)
        msg.index_finger = currents.get("index_finger", 0)
        msg.middle_finger = currents.get("middle_finger", 0)
        msg.ring_finger = currents.get("ring_finger", 0)
        msg.pinky_finger = currents.get("pinky_finger", 0)
        
        self.get_logger().info(f"Current control: {currents}")
        self.hand_cmd_pub.publish(msg)
        
        time.sleep(1.0)
        return True

    def test_hand_position_time(self, positions: dict = None, times: dict = None) -> bool:
        """测试灵巧手位置/时间混合控制"""
        self._log_section("Testing Hand Position+Time Control")
        
        if positions is None:
            positions = {
                "thumb_tip": 80,
                "thumb_base": 80,
                "index_finger": 80,
                "middle_finger": 80,
                "ring_finger": 80,
                "pinky_finger": 80,
            }
        
        if times is None:
            # 时间单位10ms，100=1秒
            times = {
                "thumb_tip": 100,
                "thumb_base": 100,
                "index_finger": 100,
                "middle_finger": 100,
                "ring_finger": 100,
                "pinky_finger": 100,
            }
        
        msg = HandPositionTimeCmd()
        msg.thumb_tip_pos = positions.get("thumb_tip", 0)
        msg.thumb_base_pos = positions.get("thumb_base", 0)
        msg.index_finger_pos = positions.get("index_finger", 0)
        msg.middle_finger_pos = positions.get("middle_finger", 0)
        msg.ring_finger_pos = positions.get("ring_finger", 0)
        msg.pinky_finger_pos = positions.get("pinky_finger", 0)
        
        msg.thumb_tip_time = times.get("thumb_tip", 0)
        msg.thumb_base_time = times.get("thumb_base", 0)
        msg.index_finger_time = times.get("index_finger", 0)
        msg.middle_finger_time = times.get("middle_finger", 0)
        msg.ring_finger_time = times.get("ring_finger", 0)
        msg.pinky_finger_time = times.get("pinky_finger", 0)
        
        self.get_logger().info(f"Position+Time control:")
        self.get_logger().info(f"  Positions: {positions}")
        self.get_logger().info(f"  Times (x10ms): {times}")
        self.hand_position_time_pub.publish(msg)
        
        time.sleep(2.0)
        return True

    def test_hand_open(self) -> bool:
        """测试灵巧手全开"""
        self._log_section("Testing Hand Open All")
        return self.test_hand_position({
            "thumb_tip": 0, "thumb_base": 0,
            "index_finger": 0, "middle_finger": 0,
            "ring_finger": 0, "pinky_finger": 0,
        })

    def test_hand_close(self) -> bool:
        """测试灵巧手全握"""
        self._log_section("Testing Hand Close All")
        return self.test_hand_position({
            "thumb_tip": 100, "thumb_base": 100,
            "index_finger": 100, "middle_finger": 100,
            "ring_finger": 100, "pinky_finger": 100,
        })

    # ==================== 综合测试 ====================

    def run_all_tests(self) -> dict:
        """运行所有测试"""
        self._log_section("RUNNING ALL TESTS")
        
        results = {}
        
        # 1. 反馈话题测试
        self.get_logger().info("\n>>> Part 1: Feedback Topics <<<")
        results["all_feedback"] = self.test_all_feedback()
        
        # 2. 服务测试
        self.get_logger().info("\n>>> Part 2: Services <<<")
        results["enable"] = self.test_enable(True)
        time.sleep(1.0)
        results["move_home"] = self.test_move_home()
        time.sleep(2.0)
        
        # 3. 机械臂控制测试
        self.get_logger().info("\n>>> Part 3: Arm Control <<<")
        results["move_j"] = self.test_move_j()
        time.sleep(1.0)
        self.test_move_home()
        time.sleep(2.0)
        results["move_p"] = self.test_move_p()
        
        # 4. 夹爪测试（如果有）
        self.get_logger().info("\n>>> Part 4: Gripper Control <<<")
        results["gripper_status"] = self.test_gripper_status(1.0)
        if results["gripper_status"]:
            results["gripper_open"] = self.test_gripper_open()
            time.sleep(0.5)
            results["gripper_close"] = self.test_gripper_close()
        
        # 5. 灵巧手测试（如果有）
        self.get_logger().info("\n>>> Part 5: Hand Control <<<")
        results["hand_status"] = self.test_hand_status(1.0)
        if results["hand_status"]:
            results["hand_open"] = self.test_hand_open()
            time.sleep(1.0)
            results["hand_close"] = self.test_hand_close()
        
        # 打印结果汇总
        self._log_section("TEST RESULTS SUMMARY")
        passed = 0
        failed = 0
        skipped = 0
        
        for test_name, result in results.items():
            if isinstance(result, dict):
                # 反馈测试返回的是计数字典
                status = "PASSED" if any(v > 0 for v in result.values()) else "NO DATA"
            elif result is True:
                status = "PASSED"
                passed += 1
            elif result is False:
                status = "FAILED/SKIPPED"
                failed += 1
            else:
                status = "UNKNOWN"
                skipped += 1
            self.get_logger().info(f"  {test_name}: {status}")
        
        self.get_logger().info(f"\nTotal: {passed} passed, {failed} failed/skipped")
        
        return results

    # ==================== 交互式菜单 ====================

    def run_interactive(self):
        """交互式测试菜单"""
        menu = """
============================================================
                    AGX ARM TEST MENU
============================================================
  [Feedback Topics]
    1.  Test all feedback topics
    2.  Test joint_states
    3.  Test end_pose
    4.  Test arm_status & arm_ctrl_states
  [Services]
    5.  Enable arm
    6.  Disable arm
    7.  Move to home
    8.  Exit teach mode
  [Arm Control]
    9.  Test move_j (joint control)
    10. Test move_p (end pose control)
    11. Test move_l (linear motion, piper)
    12. Test move_c (circular motion, piper)
    13. Test move_js (joint space, piper)
    14. Test move_mit (MIT control, piper)
  [Gripper Control]
    15. Test gripper status
    16. Gripper open
    17. Gripper close
    18. Gripper move (custom width)
  [Hand Control]
    19. Test hand status
    20. Hand open all
    21. Hand close all
    22. Hand position control
    23. Hand speed control
    24. Hand current control
    25. Hand position+time control
  [Batch Tests]
    99. Run ALL tests

    0.  Exit
============================================================
"""
        while rclpy.ok():
            try:
                print(menu)
                choice = input("Enter choice: ").strip()
                
                # Feedback tests
                if choice == "1":
                    self.test_all_feedback()
                elif choice == "2":
                    self.test_joint_states()
                elif choice == "3":
                    self.test_end_pose()
                elif choice == "4":
                    self.test_arm_status()
                    self.test_arm_ctrl_states()
                
                # Service tests
                elif choice == "5":
                    self.test_enable(True)
                elif choice == "6":
                    self.test_enable(False)
                elif choice == "7":
                    self.test_move_home()
                elif choice == "8":
                    self.test_exit_teach_mode()
                
                # Arm control tests
                elif choice == "9":
                    self.test_move_j()
                elif choice == "10":
                    self.test_move_p()
                elif choice == "11":
                    self.test_move_l()
                elif choice == "12":
                    self.test_move_c()
                elif choice == "13":
                    self.test_move_js()
                elif choice == "14":
                    self.test_move_mit()
                
                # Gripper tests
                elif choice == "15":
                    self.test_gripper_status()
                elif choice == "16":
                    self.test_gripper_open()
                elif choice == "17":
                    self.test_gripper_close()
                elif choice == "18":
                    width = float(input("Enter width (0-0.1 m): ") or "0.05")
                    force = float(input("Enter force (0-3 N): ") or "1.0")
                    self.test_gripper_move(width, force)
                
                # Hand tests
                elif choice == "19":
                    self.test_hand_status()
                elif choice == "20":
                    self.test_hand_open()
                elif choice == "21":
                    self.test_hand_close()
                elif choice == "22":
                    pos = int(input("Enter position for all fingers (0-100): ") or "50")
                    self.test_hand_position({
                        "thumb_tip": pos, "thumb_base": pos,
                        "index_finger": pos, "middle_finger": pos,
                        "ring_finger": pos, "pinky_finger": pos,
                    })
                elif choice == "23":
                    spd = int(input("Enter speed for all fingers (-100 to 100): ") or "30")
                    self.test_hand_speed({
                        "thumb_tip": spd, "thumb_base": spd,
                        "index_finger": spd, "middle_finger": spd,
                        "ring_finger": spd, "pinky_finger": spd,
                    })
                elif choice == "24":
                    cur = int(input("Enter current for all fingers (-100 to 100): ") or "20")
                    self.test_hand_current({
                        "thumb_tip": cur, "thumb_base": cur,
                        "index_finger": cur, "middle_finger": cur,
                        "ring_finger": cur, "pinky_finger": cur,
                    })
                elif choice == "25":
                    pos = int(input("Enter position for all fingers (0-100): ") or "80")
                    t = int(input("Enter time for all fingers (x10ms, 0-255): ") or "100")
                    self.test_hand_position_time(
                        positions={
                            "thumb_tip": 0, "thumb_base": 0,
                            "index_finger": pos, "middle_finger": pos,
                            "ring_finger": pos, "pinky_finger": pos,
                        },
                        times={
                            "thumb_tip": t, "thumb_base": t,
                            "index_finger": t, "middle_finger": t,
                            "ring_finger": t, "pinky_finger": t,
                        }
                    )
                
                # Batch tests
                elif choice == "99":
                    self.run_all_tests()
                
                elif choice == "0":
                    self.get_logger().info("Exiting test node...")
                    break
                else:
                    print("Invalid choice! Please enter a valid number.")
                    
            except KeyboardInterrupt:
                break
            except ValueError as e:
                print(f"Input error: {e}")
            except Exception as e:
                self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = AgxArmTestNode()
    
    try:
        time.sleep(0.5)  # 等待节点启动
        node.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
