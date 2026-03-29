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

