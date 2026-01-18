## PS4 Franka Servo

ROS 2 package that drives a Franka Panda arm with a PS4 controller using
MoveIt Servo for real-time Cartesian control, plus gripper open/close.

### Features
- PS4 joystick to twist commands (translation/rotation modes)
- Gripper open/close with a single button
- One launch file to bring up Franka, joy, and RViz

### Build
```bash
cd /home/diana/ros2_ws
colcon build --packages-select ps4_franka_servo
source /home/diana/ros2_ws/install/setup.bash
```

### Run
```bash
export ROS_LOG_DIR=/tmp/ros_log
mkdir -p /tmp/ros_log
source /home/diana/ros2_ws/install/setup.bash
ros2 launch ps4_franka_servo ps4_franka_full.launch.py
```

Optional args:
```bash
ros2 launch ps4_franka_servo ps4_franka_full.launch.py fake:=true rviz:=true
```

### Controls
- CIRCLE (index 1): enable Servo
- L1 (index 4): toggle translation/rotation
- SQUARE (index 3): toggle gripper open/close

Axes:
- axes[0] = left stick Y
- axes[1] = left stick X
- axes[3] = right stick X
- axes[2] = L2 trigger (0..1)
- axes[5] = R2 trigger (0..1)
