## Nero Command Test

```bash
ros2 topic pub /control/move_j sensor_msgs/msg/JointState "$(cat test/nero/test_move_j.yaml)" -1
```

```bash
ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped "$(cat test/nero/test_move_p.yaml)" -1
```

```bash
ros2 topic pub /control/move_c geometry_msgs/msg/PoseArray "$(cat test/nero/test_move_c.yaml)" -1
```

## Piper Command Test

```bash
ros2 topic pub /control/move_j sensor_msgs/msg/JointState "$(cat test/piper/test_move_j.yaml)" -1
```

```bash
ros2 topic pub /control/move_p geometry_msgs/msg/PoseStamped "$(cat test/piper/test_move_p.yaml)" -1
```

```bash
ros2 topic pub /control/move_l geometry_msgs/msg/PoseStamped "$(cat test/piper/test_move_l.yaml)" -1
```

```bash
ros2 topic pub /control/move_c geometry_msgs/msg/PoseArray "$(cat test/piper/test_move_c.yaml)" -1
```