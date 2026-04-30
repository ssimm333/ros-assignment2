# Search and Rescue Robot — ROS 2 Jazzy + Gazebo Harmonic

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic
- Required packages:
  ```bash
  sudo apt install ros-jazzy-ros-gz ros-jazzy-robot-state-publisher \
    ros-jazzy-controller-manager ros-jazzy-diff-drive-controller \
    ros-jazzy-joint-state-broadcaster ros-jazzy-gz-ros2-control \
    ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-mppi-controller \
    ros-jazzy-teleop-twist-keyboard
  ```

## Build

```bash
cd ~/ros2_ws   # or wherever ros-assignment2/ lives
colcon build --packages-select search_rescue_robot
source install/setup.bash
```

## Run

### Simulation only (Gazebo + robot + controllers)

```bash
ros2 launch search_rescue_robot sim.launch.py
```

This launches:
- Gazebo Harmonic with the assignment world
- Robot spawned at origin (0, 0)
- ros2_control: joint_state_broadcaster + diff_drive_controller
- ros_gz_bridge for sensors (/clock, /scan, /camera/image_raw, /imu/data)

### Navigation (Nav2 + SLAM + simulation)

```bash
ros2 launch search_rescue_robot nav2.launch.py
```

With RViz2:
```bash
ros2 launch search_rescue_robot nav2.launch.py use_rviz:=true
```

This launches everything from sim.launch.py plus:
- SLAM Toolbox (online async mapping)
- Nav2 stack (MPPI controller, NavfnPlanner, behavior server, BT navigator)
- Twist relay (converts Nav2's Twist → TwistStamped for diff_drive_controller)

In RViz2, click "Nav2 Goal" to send the robot to a point on the map.

### Test driving (manual)

In a separate terminal:
```bash
# Via diff_drive_controller (stamped twist)
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}}}" --once

# Or via teleop keyboard (requires twist_relay to be running, or use nav2.launch.py)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Verify topics

```bash
ros2 topic list
ros2 topic echo /scan                          # lidar data
ros2 topic echo /diff_drive_controller/odom    # odometry
ros2 topic echo /joint_states                  # wheel + camera joint states
ros2 topic echo /map                           # SLAM map (when nav2.launch.py is running)
```

### Check TF tree

```bash
ros2 run tf2_tools view_frames
```

Expected: map -> odom -> base_link -> (left_wheel, right_wheel, caster_wheel, lidar_link, camera_link, imu_link)
