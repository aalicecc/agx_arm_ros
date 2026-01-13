#!/usr/bin/env python3
# -*-coding:utf8-*-
import os
import time
import math
import rclpy
import threading
import numpy as np
from agx_arm.api.agx_arm_factory import create_agx_arm_config, AgxArmFactory
from trac_ik import TracIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from std_srvs.srv import SetBool, Empty
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory

class AgxArmRosNode(Node):
    def __init__(self):
        super().__init__('agx_arm_ctrl_single_node')

        # Conversion factors
        self.deg2rad = math.pi / 180.0
        self.rad2deg = 180.0 / math.pi

        # Publish frequency (Hz)
        self.pub_frequency = 200

        # Callback group
        self.callback_group = ReentrantCallbackGroup()

        # ROS parameters
        self.declare_parameters('', [
            ('can_port', 'can0'),
            ('auto_enable', False),
            ('arm_type', 'piper'),  # piper or nero
            ('end_effector_type', 'test0'),  # test0 or test1
            ('use_upper_ik', True),
            ('urdf_path', '')
        ])

        self.can_port = self.get_parameter('can_port').value
        self.enable = self.get_parameter('auto_enable').value
        self.arm_type = self.get_parameter('arm_type').value
        self.end_effector_type = self.get_parameter('end_effector_type').value
        self.use_upper_ik = self.get_parameter('use_upper_ik').value
        self.urdf_path = self.get_parameter('urdf_path').value

        self.move_speed = 100
        self.move_mode = "j"
        self.gripper_exist = False
        
        self.get_logger().info(f"can_port: {self.can_port}")
        self.get_logger().info(f"auto_enable: {self.enable}")
        self.get_logger().info(f"arm_type: {self.arm_type}")
        self.get_logger().info(f"end_effector_type: {self.end_effector_type}")
        self.get_logger().info(f"use_upper_ik: {self.use_upper_ik}")
        self.get_logger().info(f"urdf_path: {self.urdf_path}")

        # Create agx_arm config
        config = create_agx_arm_config(
            robot=self.arm_type,
            comm="can",
            can_port=self.can_port
        )

        # Create agx_arm driver instance
        self.agx_arm = AgxArmFactory.create_arm(config)
        self.agx_arm.connect(start_thread=True)
        time.sleep(0.1)

        # Create TracIK solver
        self.ik_solver = TracIK(base_link_name="base_link",
                             tip_link_name="Link8",
                             urdf_path=self.urdf_path,
                             timeout=0.005,
                             epsilon=1e-5,
                            #  solver_type="Speed",
                             solver_type="Distance",
                             )
        
        # Initialize joint state messages
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        self.joint_states = self._create_joint_state(joint_names)

        # Publishers
        self.joint_states_pub = self.create_publisher(JointState, '/feedback/joint_states', 1)
        self.end_pose_pub = self.create_publisher(PoseStamped, '/feedback/end_pose', 1)

        # Start Subscribers and Services
        self.create_subscription(JointState, '/control/joint_states', self.joint_callback, 1, callback_group=self.callback_group)
        self.create_subscription(PoseStamped, '/control/end_pose', self.pose_callback, 1, callback_group=self.callback_group)

        # Services
        self.create_service(SetBool, 'enable_agx_arm', self.enable_callback, callback_group=self.callback_group)
        self.create_service(Empty, 'move_home', self.home_callback, callback_group=self.callback_group)

        # Start publisher thread
        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()

    def JointMitCtrl(self, joints):
        """MIT控制指令"""
        for i in range(1, 8):
            joint_rad = np.deg2rad(joints[i-1] / 1000)
            self.agx_arm.move_m(joint_index=i, p_des=joint_rad, v_des=0, kp=10.0, kd=0.8, t_ff=0)

    def _create_joint_state(self, names):
        """Create a JointState message with given joint names"""
        state = JointState()
        state.name = names
        len_names = len(names)
        state.position = [0.0] * len_names
        state.velocity = [0.0] * len_names
        state.effort = [0.0] * len_names
        return state

    def _float_to_ros_time(self, timestamp: float) -> Time:
        """Convert float timestamp to ROS Time message"""
        ros_time = Time()
        ros_time.sec = int(timestamp)
        ros_time.nanosec = int((timestamp - ros_time.sec) * 1e9)
        return ros_time
    
    def _enable_arm(self, enable: bool = True):
        """Enable or disable the robotic arm"""
        if not enable:
            self.agx_arm.disable()  # Disable all joints
        else:
            self.agx_arm.enable()   # Enable all joints
    
    def _enable_gripper(self, enable: bool = True):
        """Enable or disable the gripper"""
        # TODO: Implement gripper control with agx_arm_sdk when available
        if not self.gripper_exist:
            return

        self.get_logger().warn("Gripper control not yet implemented in agx_arm_sdk")
        # if not enable:
        #     # Disable gripper logic
        # else:
        #     # Enable gripper logic

    def publish_thread(self):
        """Publish messages from the robotic arm"""
        rate = self.create_rate(self.pub_frequency)
        start_time = time.time()

        # Auto-enable check loop
        while rclpy.ok() and self.enable:
            elapsed_time = time.time() - start_time
            if elapsed_time > 5:
                self.enable = False
                self.get_logger().info("Automatic enable timeout, exiting program")
                rclpy.shutdown()
                return
            
            driver_states = self.agx_arm.get_driver_states()
            enable_flag = all(state.msg.foc_status.driver_enable_status for state in driver_states[:6] if state is not None)
            self.get_logger().info(f"Enable status: {enable_flag}")

            if enable_flag:
                break
            else:
                self._enable_arm()
                self._enable_gripper()
                time.sleep(0.5)

        # Main publishing loop
        while rclpy.ok():
            if self.agx_arm.is_connected():
                self.PublishArmJointAndGripper()
                # self.PublishArmEndPose()

            rate.sleep()

    def PublishArmJointAndGripper(self):
        """Publish joint states and gripper information"""
        joint_states = self.agx_arm.get_joint_states()
        if joint_states is None or joint_states.hz <= 0:
            return

        # Process joint positions
        joint_positions = joint_states.msg

        # Calculate fk pose
        pos, rot = self.ik_solver.fk(np.array(joint_positions))
        
        # Create pose message
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        quaternion = R.from_matrix(rot).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion

        # Update joint feedback states and publish
        self.joint_states.position = joint_positions
        self.joint_states.header.stamp = self._float_to_ros_time(joint_states.timestamp)
        self.joint_states_pub.publish(self.joint_states)

        # Publish pose
        stamped_pose = PoseStamped()
        stamped_pose.pose = pose
        stamped_pose.header.stamp = self._float_to_ros_time(joint_states.timestamp)
        self.end_pose_pub.publish(stamped_pose)

    def PublishArmEndPose(self):
        """Publish end effector pose"""
        end_pose = self.agx_arm.get_ee_pose()
        if end_pose is None or end_pose.hz <= 0:
            return

        # Create pose message
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = end_pose.msg[0:3]

        # Convert Euler angles to quaternion
        roll, pitch, yaw = end_pose.msg[3:6]
        
        # self.get_logger().info(f"End effector pose: {pose.position.x}, {pose.position.y}, {pose.position.z}, {roll * self.rad2deg}, {pitch * self.rad2deg}, {yaw * self.rad2deg}")
        
        quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        
        # Publish pose
        stamped_pose = PoseStamped()
        stamped_pose.pose = pose
        stamped_pose.header.stamp = self._float_to_ros_time(end_pose.timestamp)
        self.end_pose_pub.publish(stamped_pose)

    def joint_callback(self, joint_data: JointState):
        """Callback function for joint angles"""
        joint_states = self.agx_arm.get_joint_states()
        if joint_states is None or joint_states.hz <= 0:
            self.get_logger().warn("Agx_arm is not connected, cannot control joints")
            return

        if not self.enable:
            self.get_logger().warn("Agx_arm is not enabled, cannot control joints")
            return

        # Get joint positions, velocities, and efforts
        joint_pos, joint_vel, joint_eff = {}, {}, {}
        for idx, joint_name in enumerate(joint_data.name):
            if 'joint' in joint_name:
                if len(joint_data.position) >= idx + 1:
                    joint_pos[joint_name] = 0.0 if joint_data.position[idx] != joint_data.position[idx] else joint_data.position[idx]
                if len(joint_data.velocity) >= idx + 1:
                    joint_vel[joint_name] = 0.0 if joint_data.velocity[idx] != joint_data.velocity[idx] else joint_data.velocity[idx]
                if len(joint_data.effort) >= idx + 1:
                    joint_eff[joint_name] = 0.0 if joint_data.effort[idx] != joint_data.effort[idx] else joint_data.effort[idx]
            elif 'gripper' in joint_name:
                if len(joint_data.position) >= idx + 1:
                    joint_pos[joint_name] = 0.0 if joint_data.position[idx] != joint_data.position[idx] else joint_data.position[idx]
                if len(joint_data.effort) >= idx + 1:
                    joint_eff[joint_name] = 0.0 if joint_data.effort[idx] != joint_data.effort[idx] else joint_data.effort[idx]

        # Control joints
        arm_status = self.agx_arm.get_arm_status()
        if arm_status is not None and arm_status.msg.ctrl_mode == 0x02:
            self.get_logger().warn("Cannot control joints in teach mode")
            return
        else:
            joints = [round(joint_pos.get(f'joint{i}', 0) * self.rad2deg * 1e3) for i in range(1, 8)]
            self.agx_arm.set_speed(self.move_speed)
            self.agx_arm.set_motion_mode(1, 0xAD)  # move_js mode
            self.JointMitCtrl(joints)

    def pose_callback(self, pose_data: PoseStamped):
        """Callback function for end effector pose"""
        joint_states = self.agx_arm.get_joint_states()
        if joint_states is None or joint_states.hz <= 0:
            self.get_logger().warn("Agx_arm is not connected, cannot control end effector pose")
            return

        if not self.enable:
            self.get_logger().warn("Agx_arm is not enabled, cannot control end effector pose")
            return

        arm_status = self.agx_arm.get_arm_status()
        if arm_status is not None and arm_status.msg.ctrl_mode == 0x02:
            self.get_logger().warn("Cannot control end effector pose in teach mode")
            return

        # Convert quaternion to Euler angles
        quaternion = [pose_data.pose.orientation.x,
                      pose_data.pose.orientation.y,
                      pose_data.pose.orientation.z,
                      pose_data.pose.orientation.w]

        if not self.use_upper_ik:
            # Direct pose control - convert to required format for agx_arm_sdk
            pose_xyz = [pose_data.pose.position.x, pose_data.pose.position.y, pose_data.pose.position.z]
            euler_angles = R.from_quat(quaternion).as_euler('xyz', degrees=False)  # radians
            pose_cmd = pose_xyz + euler_angles.tolist()

            # Control end effector pose
            self.agx_arm.set_speed(self.move_speed)
            self.agx_arm.set_motion_mode(0, 0)  # move_p mode
            self.agx_arm.move_p(pose_cmd)
        else:
            tgt_pos = np.array([pose_data.pose.position.x, pose_data.pose.position.y, pose_data.pose.position.z])
            tgt_rot = R.from_quat(quaternion).as_matrix()

            # seed = np.zeros(7)
            seed = self.joint_states.position
            res = self.ik_solver.ik(tgt_pos, tgt_rot, seed_jnt_values=seed)
            if res is None or len(res) != 7:
                self.get_logger().warn("Failed to find IK solution")
                return

            res = [round(x * self.rad2deg * 1e3) for x in res]
            self.agx_arm.set_speed(self.move_speed)
            self.agx_arm.set_motion_mode(1, 0xAD)  # move_js mode
            self.JointMitCtrl(res)

    def enable_callback(self, request, response):
        """Service callback function for enabling the robotic arm"""
        self.get_logger().info(f"Received enable request: {request.data}")
        
        try:
            joint_states = self.agx_arm.get_joint_states()
            if joint_states is None or joint_states.hz <= 0:
                response.success = False
                response.message = "Agx_arm is not connected, cannot set enable state"
            elif request.data:
                self._enable_arm()
                self._enable_gripper()
                self.enable = True
                response.success = True
                response.message = "Agx_arm enabled successfully"
            else:
                self._enable_arm(False)
                self._enable_gripper(False)
                self.enable = False
                response.success = True
                response.message = "Agx_arm disabled successfully"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to set enable state: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

    def home_callback(self, request, response):
        """Service callback function for moving the robotic arm to home position"""
        self.get_logger().info("Received move home request")
        
        try:
            joint_states = self.agx_arm.get_joint_states()
            if joint_states is None or joint_states.hz <= 0:
                self.get_logger().warn("Agx_arm is not connected, cannot move to home position")
            elif self.enable:
                arm_status = self.agx_arm.get_arm_status()
                if arm_status is not None and arm_status.msg.ctrl_mode == 0x02:
                    self.get_logger().warn("Cannot move to home position in teach mode")
                    return response
                else:
                    self.agx_arm.set_speed(self.move_speed)
                    self.agx_arm.set_motion_mode(1, 0xAD)  # move_js mode
                    self.JointMitCtrl([0] * 7)
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