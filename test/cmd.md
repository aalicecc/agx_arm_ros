```bash
ros2 topic pub /control/joint_states sensor_msgs/msg/JointState "$(cat test/test_joint_states.yaml)" -1
```

```bash
ros2 topic pub /control/end_pose geometry_msgs/msg/PoseStamped "$(cat test/test_end_pose.yaml)" -1
```