# ROS Assignment 2
Search and Rescue Robot Project

Assignment Details can be found here -> [here](SearchandRescueRobotProject.pdf)

# Installation

## System prep
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release software-properties-common \
  build-essential cmake git python3-pip python3-colcon-common-extensions \
  python3-rosdep python3-vcstool
```

## install Ros2 Jazzy
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

## Add Gazebo Harmonic
```bash
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
sudo apt update
sudo apt install -y gz-harmonic
```

## Ros2 + Gazebo Bridge and Project Dependencies
```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-nav2-bringup \
  ros-jazzy-navigation2 \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-behaviortree-cpp \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  python3-colcon-common-extensions
```

## Source Ros2 automatically
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export GZ_VERSION=harmonic" >> ~/.bashrc
source ~/.bashrc
```

## Init Rosdep
```bash
sudo rosdep init
rosdep update
```

## Create workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

# Project layout (this repository)

This repository itself is a ROS 2 workspace root. The expected structure is:

```text
A2/
  src/
    search_rescue_robot/
      package.xml
      setup.py
      launch/
      config/
      urdf/
      worlds/
  build/
  install/
  log/
```

# Build and run this project

Use Linux or WSL with ROS 2 Jazzy installed.

```bash
cd /path/to/A2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select search_rescue_robot
source install/setup.bash
```

Run full simulation and mission stack:

```bash
ros2 launch search_rescue_robot full_system.launch.py
```

Run only simulation + robot spawn:

```bash
ros2 launch search_rescue_robot sim_spawn.launch.py
```

Run only mission controller (if simulation already running):

```bash
ros2 launch search_rescue_robot mission.launch.py
```

Check mission status service:

```bash
ros2 service call /mission_controller/get_mission_status std_srvs/srv/Trigger "{}"
```

View mission events:

```bash
ros2 topic echo /mission/events
```

# Assignment requirement coverage

## Part 1 - Robot modelling
- URDF model: `src/search_rescue_robot/urdf/robot.urdf`
- Includes base, differential wheels, caster, lidar, camera pan joint, IMU link

## Part 2 - Spawn robot in Gazebo
- Launch file: `src/search_rescue_robot/launch/sim_spawn.launch.py`
- Spawns with robot_state_publisher and `ros_gz_sim create` at x=0, y=0

## Part 3 - Gazebo plugins and sensors
- URDF sensor blocks for camera, lidar, and IMU
- Gazebo and ROS bridge configured in `sim_spawn.launch.py`

## Part 4 - ROS 2 Control
- Control config: `src/search_rescue_robot/config/ros2_control.yaml`
- ros2_control interfaces in URDF and controller spawners in simulation launch

## Part 5 - Behaviour Tree and mission logic
- Mission node: `src/search_rescue_robot/search_rescue_robot/mission_controller.py`
- Behavior tree primitives: `src/search_rescue_robot/search_rescue_robot/behavior_tree.py`
- Includes sequence, fallback, timeout decorator, and asynchronous navigation action usage
- Handles:
  - Survivor + medical kit task
  - Dam scan with camera pan left/right
  - Low battery docking behavior
  - Fire safety distance check
  - Exit task
 - Navigation is custom waypoint-based, using odometry plus direct velocity control rather than Nav2.

## Report template
- Fill and submit: `REPORT_TEMPLATE.md`

# Notes

- The simulation world is packaged at `src/search_rescue_robot/worlds/assignment_world.sdf`.
- `ros2_control` settings are in `src/search_rescue_robot/config/ros2_control.yaml`.
- Velocity commands are sent to `/diff_drive_controller/cmd_vel_unstamped`.
- If `colcon` is not found, install `python3-colcon-common-extensions` in your ROS environment.
- On Windows PowerShell, you should run ROS 2 Linux workflows inside WSL where Jazzy and Gazebo are installed.

