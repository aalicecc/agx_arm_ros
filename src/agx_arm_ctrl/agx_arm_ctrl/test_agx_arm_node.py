#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
AGX机械臂ROS2节点测试脚本

功能：
1. 订阅反馈话题验证数据
2. 发布控制命令测试
3. 调用服务测试

使用方法：
    ros2 run agx_arm_ctrl test_agx_arm_node
"""

import time
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import SetBool, Empty
from scipy.spatial.transform import Rotation as R

from agx_arm_msgs.msg import AgxArmStatus, TriplePose


class AgxArmTestNode(Node):
    def __init__(self):
        super().__init__("agx_arm_test_node")

        # 订阅反馈话题
        self.joint_states_sub = self.create_subscription(
            JointState, "/feedback/joint_states", self._joint_states_callback, 1
        )
        self.end_pose_sub = self.create_subscription(
            PoseStamped, "/feedback/end_pose", self._end_pose_callback, 1
        )
        self.arm_status_sub = self.create_subscription(
            AgxArmStatus, "/feedback/arm_status", self._arm_status_callback, 1
        )

        # 控制话题发布者
        self.move_j_pub = self.create_publisher(JointState, "/control/move_j", 1)
        self.move_p_pub = self.create_publisher(PoseStamped, "/control/move_p", 1)
        self.move_l_pub = self.create_publisher(PoseStamped, "/control/move_l", 1)
        self.move_c_pub = self.create_publisher(TriplePose, "/control/move_c", 1)
        self.move_js_pub = self.create_publisher(JointState, "/control/move_js", 1)
        self.move_mit_pub = self.create_publisher(JointState, "/control/move_mit", 1)

        # 服务客户端
        self.enable_client = self.create_client(SetBool, "enable_agx_arm")
        self.move_home_client = self.create_client(Empty, "move_home")
        self.exit_teach_client = self.create_client(Empty, "exit_teach_mode")

        # 状态变量
        self.last_joint_states = None
        self.last_end_pose = None
        self.last_arm_status = None
        self.receive_count = 0

        self.get_logger().info("Test node initialized")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Available commands:")
        self.get_logger().info("  1. test_feedback  - Test feedback topics")
        self.get_logger().info("  2. test_enable    - Test enable/disable")
        self.get_logger().info("  3. test_home      - Test move to home")
        self.get_logger().info("  4. test_move_j    - Test move_j control")
        self.get_logger().info("  5. test_move_p    - Test move_p control")
        self.get_logger().info("  6. test_all       - Run all tests")
        self.get_logger().info("=" * 50)

    # ==================== 回调函数 ====================

    def _joint_states_callback(self, msg: JointState):
        self.last_joint_states = msg
        self.receive_count += 1

    def _end_pose_callback(self, msg: PoseStamped):
        self.last_end_pose = msg

    def _arm_status_callback(self, msg: AgxArmStatus):
        self.last_arm_status = msg

    # ==================== 测试方法 ====================

    def test_feedback(self, duration: float = 2.0):
        """测试反馈话题"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Testing feedback topics...")
        
        self.receive_count = 0
        start_time = time.time()
        
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info(f"Received {self.receive_count} joint state messages in {duration}s")
        self.get_logger().info(f"Frequency: {self.receive_count / duration:.1f} Hz")
        
        if self.last_joint_states is not None:
            self.get_logger().info(f"Joint names: {self.last_joint_states.name}")
            positions = [f"{p:.4f}" for p in self.last_joint_states.position]
            self.get_logger().info(f"Joint positions (rad): {positions}")
        else:
            self.get_logger().warn("No joint states received!")
        
        if self.last_end_pose is not None:
            p = self.last_end_pose.pose.position
            o = self.last_end_pose.pose.orientation
            self.get_logger().info(f"End pose position: x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}")
            self.get_logger().info(f"End pose orientation: x={o.x:.4f}, y={o.y:.4f}, z={o.z:.4f}, w={o.w:.4f}")
        else:
            self.get_logger().warn("No end pose received!")
        
        if self.last_arm_status is not None:
            self.get_logger().info(f"Arm status: ctrl_mode={self.last_arm_status.ctrl_mode}, "
                                   f"arm_status={self.last_arm_status.arm_status}, "
                                   f"motion_status={self.last_arm_status.motion_status}")
        else:
            self.get_logger().warn("No arm status received!")
        
        return self.receive_count > 0

    def test_enable(self, enable: bool = True):
        """测试使能/失能"""
        self.get_logger().info("=" * 50)
        action = "enable" if enable else "disable"
        self.get_logger().info(f"Testing {action} arm...")
        
        if not self.enable_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Enable service not available!")
            return False
        
        request = SetBool.Request()
        request.data = enable
        
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f"{action.capitalize()} result: success={result.success}")
            return result.success
        else:
            self.get_logger().error(f"{action.capitalize()} service call failed!")
            return False

    def test_move_home(self):
        """测试回零"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Testing move to home position...")
        
        if not self.move_home_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Move home service not available!")
            return False
        
        request = Empty.Request()
        future = self.move_home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        self.get_logger().info("Move home command sent")
        
        # 等待运动完成
        self.get_logger().info("Waiting for motion to complete...")
        time.sleep(3.0)
        
        return True

    def test_move_j(self, joint_angles: list = None):
        """测试move_j关节控制"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Testing move_j control...")
        
        # 先获取当前关节数量
        if self.last_joint_states is None:
            self.test_feedback(1.0)
        
        if self.last_joint_states is None:
            self.get_logger().error("Cannot get current joint states!")
            return False
        
        joint_count = len(self.last_joint_states.name)
        
        if joint_angles is None:
            # 默认测试：小幅度运动
            joint_angles = [0.0, 0.4, -0.4, 0.0, -0.4, 0.0][:joint_count]
            if joint_count > 6:
                joint_angles.extend([0.0] * (joint_count - 6))
        
        msg = JointState()
        msg.name = [f"joint{i}" for i in range(1, joint_count + 1)]
        msg.position = joint_angles
        
        self.get_logger().info(f"Sending move_j command: {joint_angles}")
        self.move_j_pub.publish(msg)
        
        # 等待运动
        time.sleep(2.0)
        
        # 检查是否到达
        self.test_feedback(0.5)
        if self.last_joint_states is not None:
            current = list(self.last_joint_states.position)
            errors = [abs(c - t) for c, t in zip(current, joint_angles)]
            max_error = max(errors)
            self.get_logger().info(f"Max position error: {max_error:.4f} rad")
        
        return True

    def test_move_p(self, pose: list = None):
        """测试move_p末端控制"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Testing move_p control...")
        
        # 先获取当前末端位姿
        if self.last_end_pose is None:
            self.test_feedback(1.0)
        
        if self.last_end_pose is None:
            self.get_logger().error("Cannot get current end pose!")
            return False
        
        if pose is None:
            # 默认测试：在当前位置基础上小幅移动
            current = self.last_end_pose.pose
            pose = [
                current.position.x,
                current.position.y,
                current.position.z + 0.02,  # 向上移动2cm
            ]
            # 保持当前姿态
            quat = [
                current.orientation.x,
                current.orientation.y,
                current.orientation.z,
                current.orientation.w,
            ]
        else:
            # pose格式: [x, y, z, roll, pitch, yaw]
            quat = R.from_euler("xyz", pose[3:6]).as_quat()
            pose = pose[:3]
        
        msg = PoseStamped()
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.get_logger().info(f"Sending move_p command: position=[{pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.4f}]")
        self.move_p_pub.publish(msg)
        
        time.sleep(2.0)
        return True

    def test_move_l(self, pose: list = None):
        """测试move_l直线运动"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Testing move_l control (piper only)...")
        
        if self.last_end_pose is None:
            self.test_feedback(1.0)
        
        if self.last_end_pose is None:
            self.get_logger().error("Cannot get current end pose!")
            return False
        
        if pose is None:
            current = self.last_end_pose.pose
            pose = [
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
            quat = R.from_euler("xyz", pose[3:6]).as_quat()
            pose = pose[:3]
        
        msg = PoseStamped()
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.get_logger().info(f"Sending move_l command: position=[{pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.4f}]")
        self.move_l_pub.publish(msg)
        
        time.sleep(2.0)
        return True

    def test_move_c(self, start_pose: PoseStamped = None, mid_pose: PoseStamped = None, end_pose: PoseStamped = None):
        """测试move_c圆弧运动"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Testing move_c control...")
        
        if self.last_end_pose is None:
            self.test_feedback(1.0)
        
        if self.last_end_pose is None:
            self.get_logger().error("Cannot get current end pose!")
            return False
        
        current = self.last_end_pose.pose
        
        if start_pose is None:
            start_pose = PoseStamped()
            start_pose.pose.position.x = 0.2
            start_pose.pose.position.y = 0.0
            start_pose.pose.position.z = 0.3
            start_pose.pose.orientation = current.orientation
        
        if mid_pose is None:
            mid_pose = PoseStamped()
            mid_pose.pose.position.x = 0.2
            mid_pose.pose.position.y = 0.05
            mid_pose.pose.position.z = 0.35
            mid_pose.pose.orientation = current.orientation
        
        if end_pose is None:
            end_pose = PoseStamped()
            end_pose.pose.position.x = 0.2
            end_pose.pose.position.y = 0.0
            end_pose.pose.position.z = 0.4
            end_pose.pose.orientation = current.orientation
        
        msg = TriplePose()
        msg.start_pose = start_pose
        msg.mid_pose = mid_pose
        msg.end_pose = end_pose
        
        self.get_logger().info("Sending move_c command")
        self.move_c_pub.publish(msg)
        
        time.sleep(2.0)
        return True

    def test_js(self, joint_angles: list = None):
        """测试move_j关节空间运动"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Testing move_j control...")
        if self.last_joint_states is None:
            self.test_feedback(1.0)

        if self.last_joint_states is None:
            self.get_logger().error("Cannot get current joint states!")
            return False

        joint_count = len(self.last_joint_states.name)
        
        if joint_angles is None:
            # 默认测试：小幅度运动
            joint_angles = [0.0, 0.4, -0.4, 0.0, -0.4, 0.0][:joint_count]
            if joint_count > 6:
                joint_angles.extend([0.0] * (joint_count - 6))
        
        msg = JointState()
        msg.name = [f"joint{i}" for i in range(1, joint_count + 1)]
        msg.position = joint_angles

        self.get_logger().info(f"Sending move_js command: {joint_angles}")
        self.move_js_pub.publish(msg)

    def test_mit(self):
        """测试MIT控制"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Testing MIT control...")
        
        if self.last_joint_states is None:
            self.test_feedback(1.0)
        
        if self.last_joint_states is None:
            self.get_logger().error("Cannot get current joint states!")
            return False
        
        joint_count = len(self.last_joint_states.name)
        joint_rad = [0.0, 0.4, -0.4, 0.0, -0.4, 0.0][:joint_count]
        
        self.get_logger().info(f"Sending MIT command to move joints to zero position")

        msg = JointState()
        msg.name = self.last_joint_states.name
        msg.position = joint_rad

        self.move_mit_pub.publish(msg)

        time.sleep(2.0)
        return True


    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Running all tests...")
        self.get_logger().info("=" * 50)
        
        results = {}
        
        # 1. 测试反馈
        results["feedback"] = self.test_feedback()
        
        # 2. 测试使能
        results["enable"] = self.test_enable(True)
        time.sleep(1.0)
        
        # 3. 测试回零
        results["move_home"] = self.test_move_home()
        time.sleep(2.0)
        
        # 4. 测试move_j
        results["move_j"] = self.test_move_j()
        time.sleep(1.0)
        
        # 5. 回零
        self.test_move_home()
        time.sleep(2.0)
        
        # 6. 测试move_p
        results["move_p"] = self.test_move_p()
        
        # 打印结果
        self.get_logger().info("=" * 50)
        self.get_logger().info("Test Results:")
        for test_name, passed in results.items():
            status = "PASSED" if passed else "FAILED"
            self.get_logger().info(f"  {test_name}: {status}")
        self.get_logger().info("=" * 50)
        
        return all(results.values())

    def run_interactive(self):
        """交互式测试"""
        while rclpy.ok():
            try:
                print("\n" + "=" * 50)
                print("Select test:")
                print("  1. Test feedback topics")
                print("  2. Enable arm")
                print("  3. Disable arm")
                print("  4. Move to home")
                print("  5. Test move_j")
                print("  6. Test move_p")
                print("  7. Test move_l (piper only)")
                print("  8. Run all tests")
                print("  9. Show current status")
                print(" 10. Test move_c")
                print(" 11. Test MIT control")
                print(" 12. Test joint space control")
                print("  0. Exit")
                print("=" * 50)
                
                choice = input("Enter choice (0-9): ").strip()
                
                if choice == "1":
                    self.test_feedback()
                elif choice == "2":
                    self.test_enable(True)
                elif choice == "3":
                    self.test_enable(False)
                elif choice == "4":
                    self.test_move_home()
                elif choice == "5":
                    self.test_move_j()
                elif choice == "6":
                    self.test_move_p()
                elif choice == "7":
                    self.test_move_l()
                elif choice == "8":
                    self.run_all_tests()
                elif choice == "9":
                    self.test_feedback(0.5)
                elif choice == "10":
                    self.test_move_c()
                elif choice == "11":
                    self.test_mit()
                elif choice == "12":
                    self.test_js()
                elif choice == "0":
                    break
                else:
                    print("Invalid choice!")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = AgxArmTestNode()
    
    try:
        # 先等待一下确保节点启动
        time.sleep(1.0)
        
        # 运行交互式测试
        node.run_interactive()
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

