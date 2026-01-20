#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
import rclpy
import math
import threading
from pyAgxArm import create_agx_arm_config, AgxArmFactory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from std_srvs.srv import SetBool, Empty
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R

class AgxArmRosNode(Node):
    def __init__(self):
        super().__init__('agx_arm_ctrl_single_node')

        ### ros parameters
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('pub_rate', 200)
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('arm_type', 'piper')
        self.declare_parameter('speed_percent', 100)
        self.declare_parameter('enable_timeout', 5.0)

        self.can_port = self.get_parameter('can_port').value
        self.pub_rate = self.get_parameter('pub_rate').value
        self.auto_enable = self.get_parameter('auto_enable').value
        self.arm_type = self.get_parameter('arm_type').value
        self.speed_percent = self.get_parameter('speed_percent').value
        self.enable_timeout = self.get_parameter('enable_timeout').value

        self.get_logger().info(f"can_port: {self.can_port}")
        self.get_logger().info(f"pub_rate: {self.pub_rate}")
        self.get_logger().info(f"auto_enable: {self.auto_enable}")
        self.get_logger().info(f"arm_type: {self.arm_type}")
        self.get_logger().info(f"speed_percent: {self.speed_percent}")
        self.get_logger().info(f"enable_timeout: {self.enable_timeout}")

        ### variables
        self.enable_flag = False

        ### publishers
        self.joint_states_pub = self.create_publisher(JointState, '/feedback/joint_states', 1)
        self.end_pose_pub = self.create_publisher(PoseStamped, '/feedback/end_pose', 1)
        # TODO:
        # arm_status_pub
        # joint_ctrl

        ### subscribers
        self.create_subscription(JointState, '/control/joint_states', self.joint_callback, 1)
        self.create_subscription(PoseStamped, '/control/end_pose', self.pose_callback, 1)

        ### services
        self.create_service(SetBool, 'enable_agx_arm', self.enable_callback)
        self.create_service(Empty, 'move_home', self.home_callback)

        ### publisher thread
        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()

        ### AgxArmFactory
        config = create_agx_arm_config(
            robot=self.arm_type,
            comm="can",
            channel=self.can_port
        )
        self.agx_arm = AgxArmFactory.create_arm(config)
        self.agx_arm.connect()

    def _float_to_ros_time(self, timestamp: float) -> Time:
        """Convert float timestamp to ROS Time message"""
        ros_time = Time()
        ros_time.sec = int(timestamp)
        ros_time.nanosec = int((timestamp - ros_time.sec) * 1e9)
        return ros_time

    def _safe_get_value(self, array, index, default=0.0):
        if index >= len(array):
            return default
        value = array[index]
        return default if math.isnan(value) else value
    
    def _enable_arm(self, enable: bool = True, timeout: float = 5.0) -> bool:
        start_time = time.time()
        if not enable:
            while not self.agx_arm.disable():
                if time.time() - start_time > timeout:
                    self.get_logger().error(f"Timeout waiting for arm to disable after {timeout} seconds")
                    return False
                time.sleep(0.01)
            self.get_logger().info("Arm disabled successfully")
            return True
        else:
            while not self.agx_arm.enable():
                if time.time() - start_time > timeout:
                    self.get_logger().error(f"Timeout waiting for arm to enable after {timeout} seconds")
                    return False
                time.sleep(0.01)
            self.get_logger().info("Arm enabled successfully")
            return True

    
    def publish_thread(self):
        rate = self.create_rate(self.pub_rate)

        while rclpy.ok() and self.auto_enable:
            enable_flag = all(self.agx_arm.get_joints_enable_status_list())
            # self.get_logger().info(f"All joints enable status is {enable_flag}")

            if enable_flag:
                break
            else:
                enable_success = self._enable_arm(True)
                if not enable_success:
                    self.get_logger().error("Failed to auto-enable the arm")
                    break

        # publishing loop
        while rclpy.ok():
            if self.agx_arm.is_ok():
                self.publish_arm_joint_and_gripper_states()
                # self.publish_arm_end_pose()

            rate.sleep()

    def publish_arm_joint_states(self):
        joint_states = self.agx_arm.get_joint_states()
        if joint_states is None or joint_states.hz <= 0:
            return
        
        len_names = len(self.joint_names)

        self.joint_states = JointState()
        self.joint_states.name = self.joint_names
        self.joint_states.position = [0.0] * len_names
        self.joint_states.velocity = [0.0] * len_names
        self.joint_states.effort = [0.0] * len_names
        self.joint_states.position = joint_states.msg
        self.joint_states.header.stamp = self._float_to_ros_time(joint_states.timestamp)
        self.joint_states_pub.publish(self.joint_states)

    def publish_arm_gripper_state(self):
        pass

    def publish_arm_joint_and_gripper_states(self):
        self.publish_arm_joint_states()
        self.publish_arm_gripper_state()

    def publish_arm_end_pose(self):
        end_pose = self.agx_arm.get_ee_pose()
        if end_pose is None or end_pose.hz <= 0:
            return

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = end_pose.msg[0:3]
        roll, pitch, yaw = end_pose.msg[3:6]
        quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        # self.get_logger().info(f"End effector pose: {pose.position.x}, {pose.position.y}, {pose.position.z}, {roll * self.rad2deg}, {pitch * self.rad2deg}, {yaw * self.rad2deg}")
        
        stamped_pose = PoseStamped()
        stamped_pose.pose = pose
        stamped_pose.header.stamp = self._float_to_ros_time(end_pose.timestamp)
        self.end_pose_pub.publish(stamped_pose)

    def joint_callback(self, joint_state: JointState):
        joint_states = self.agx_arm.get_joint_states()
        if joint_states is None or joint_states.hz <= 0 or not self.enable_flag:
            self.get_logger().warn("Agx_arm is not connected or not enabled, cannot control end effector pose")
            return

        joint_pos, joint_vel, joint_eff = {}, {}, {}
        for idx, joint_name in enumerate(joint_state.name):
            joint_pos[joint_name] = self._safe_get_value(joint_state.position, idx)
            joint_vel[joint_name] = self._safe_get_value(joint_state.velocity, idx)
            joint_eff[joint_name] = self._safe_get_value(joint_state.effort, idx)
            
        # control joints
        arm_status = self.agx_arm.get_arm_status()
        if arm_status is not None and arm_status.msg.ctrl_mode == 0x02:
            self.get_logger().warn("Cannot control joints in teach mode")
            return
        else:
            self.agx_arm.set_speed_percent(self.speed_percent)
            self.agx_arm.set_motion_mode("j")
            self.agx_arm.move_j([joint_pos.get(f'joint{i}', 0) for i in range(1, self.joint_count + 1)])
            
    def pose_callback(self, pose_data: PoseStamped):
        joint_states = self.agx_arm.get_joint_states()
        if joint_states is None or joint_states.hz <= 0 or not self.enable_flag:
            self.get_logger().warn("Agx_arm is not connected or not enabled, cannot control end effector pose")
            return

        arm_status = self.agx_arm.get_arm_status()
        if arm_status is not None and arm_status.msg.ctrl_mode == 0x02:
            self.get_logger().warn("Cannot control end effector pose in teach mode")
            return

        quaternion = [pose_data.pose.orientation.x,
                      pose_data.pose.orientation.y,
                      pose_data.pose.orientation.z,
                      pose_data.pose.orientation.w]
        pose_xyz = [pose_data.pose.position.x, pose_data.pose.position.y, pose_data.pose.position.z]
        euler_angles = R.from_quat(quaternion).as_euler('xyz', degrees=False)  # radians
        pose_cmd = pose_xyz + euler_angles.tolist()

        # control end effector pose
        self.agx_arm.set_speed_percent(self.speed_percent)
        self.agx_arm.set_motion_mode("p")  # move_p mode
        self.agx_arm.move_p(pose_cmd)
        
    def enable_callback(self, request, response):
        # self.get_logger().info(f"Received enable request: {request.data}")
        
        try:
            joint_states = self.agx_arm.get_joint_states()
            if joint_states is None or joint_states.hz <= 0:
                response.success = False
                response.message = "Agx_arm is not connected, cannot set enable state"
            elif request.data:
                enable_success = self._enable_arm(True)
                if enable_success:
                    self.enable_flag = all(self.agx_arm.get_joints_enable_status_list())
                    response.success = True
                    response.message = "Agx_arm enabled successfully"
                else:
                    response.success = False
                    response.message = "Failed to enable Agx_arm"
            else:
                disable_success = self._enable_arm(False)
                if disable_success:
                    self.enable_flag = all(self.agx_arm.get_joints_enable_status_list())
                    response.success = True
                    response.message = "Agx_arm disabled successfully"
                else:
                    response.success = False
                    response.message = "Failed to disable Agx_arm"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to set enable state: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def home_callback(self, request, response):
        # self.get_logger().info("Received move home request")
        
        try:
            joint_states = self.agx_arm.get_joint_states()
            if joint_states is None or joint_states.hz <= 0:
                self.get_logger().warn("Agx_arm is not connected, cannot move to home position")
            elif self.enable_flag:
                arm_status = self.agx_arm.get_arm_status()
                if arm_status is not None and arm_status.msg.ctrl_mode == 0x02:
                    self.get_logger().warn("Cannot move to home position in teach mode")
                    return response
                else:
                    self.agx_arm.set_speed_percent(self.speed_percent)
                    self.agx_arm.set_motion_mode("j")
                    self.agx_arm.move_j([0] * self.joint_count)
                self.get_logger().info("Agx_arm moved to home position successfully")
            else:
                self.get_logger().warn("Agx_arm is not enabled, cannot move to home position")
        except Exception as e:
            self.get_logger().error(f"Failed to move to home position: {str(e)}")
            
        return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AgxArmRosNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()