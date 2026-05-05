# Search and Rescue Robot - Project Report

## Section 1 - ROS Architecture

### Package Structure

The project is split into two ROS 2 packages:

- **`search_rescue_robot`** (ament_python) - Contains the robot URDF, Gazebo world, launch files, YAML configuration, and two Python utility nodes (`battery_simulator` and `twist_relay`).
- **`rescue_bt`** (ament_cmake) - Contains the BehaviorTree.CPP mission controller written in C++. This package defines all custom BT nodes and the main `mission_bt_node` executable, along with the XML tree definition (`rescue_mission.xml`).

### Nodes

| Node | Package | Language | Purpose |
|------|---------|----------|---------|
| `robot_state_publisher` | `robot_state_publisher` | C++ | Reads the URDF and publishes the full TF tree (all link frames) so other nodes know where every part of the robot is. |
| `ros_gz_sim` (Gazebo) | `ros_gz_sim` | C++ | Runs Gazebo Harmonic with the assignment world loaded. The robot is spawned via `ros_gz_sim create` using the `-topic robot_description` argument, which reads the URDF from the `/robot_description` topic. |
| `ros_gz_bridge` | `ros_gz_bridge` | C++ | Bridges four Gazebo topics into ROS 2: `/clock` (simulation time), `/scan` (LiDAR), `/camera/image_raw` (camera), and `/imu/data` (IMU). Without this bridge, sensor data from Gazebo would not be accessible to ROS nodes. |
| `controller_manager` | `ros2_control` | C++ | Manages the ros2_control hardware interface. It loads two controllers: `joint_state_broadcaster` (publishes joint positions/velocities) and `diff_drive_controller` (accepts velocity commands and computes wheel velocities). The controllers are spawned sequentially using `OnProcessExit` event handlers in the launch file - the joint state broadcaster starts after the robot entity is spawned, and the diff drive controller starts after the joint state broadcaster is ready. |
| `twist_relay` | `search_rescue_robot` | Python | A simple relay node. Nav2 outputs velocity commands as `geometry_msgs/Twist` on `/cmd_vel`, but `diff_drive_controller` expects `geometry_msgs/TwistStamped` on `/diff_drive_controller/cmd_vel`. This node subscribes to one and republishes on the other, adding a timestamp and `base_link` frame ID. |
| `battery_simulator` | `search_rescue_robot` | Python | Simulates battery drain and recharge. It runs at 1 Hz, draining the battery by 0.05%/s when undocked and charging at 2.0%/s when docked. It publishes a `std_msgs/Bool` on `/battery_level_low`. The node uses hysteresis logic: the flag goes `true` when the level drops to 20% and only goes back to `false` once it reaches 90%. This prevents the robot from constantly toggling between docking and resuming the mission. It subscribes to `/is_docked` (published by the BT) to know when to switch from draining to charging. All thresholds are configurable via ROS parameters (`drain_rate`, `charge_rate`, `low_threshold`, `full_threshold`). |
| `slam_toolbox` | `slam_toolbox` | C++ | Runs online asynchronous SLAM using 2D LiDAR scans. Builds a map of the environment on the fly and publishes the `map -> odom` transform so the robot can localise itself in the world frame. |
| Nav2 stack | `nav2_bringup` | C++ | The full Nav2 navigation stack, consisting of `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `velocity_smoother`, `bt_navigator`, and `waypoint_follower`. These are all lifecycle-managed nodes started by `lifecycle_manager_navigation`. |
| `mission_bt_node` | `rescue_bt` | C++ | The main mission controller. Loads the behavior tree XML, registers all custom BT node types, places the ROS node on the BT blackboard, and then ticks the tree at 10 Hz in a loop. It waits for the Nav2 `navigate_to_pose` action server to be available before starting, plus an additional 5-second delay to ensure Nav2's lifecycle activation is fully complete. |

### Topics

| Topic | Message Type | Publisher | Subscriber | Purpose |
|-------|-------------|-----------|------------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 velocity smoother | `twist_relay` | Nav2's velocity output after smoothing. |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | `twist_relay` | `diff_drive_controller` | Stamped velocity commands that the diff drive controller actually consumes. |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | `diff_drive_controller` | Nav2, SLAM Toolbox | Wheel odometry used for localisation and navigation. |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo (via bridge) | SLAM Toolbox, Nav2 costmaps | 360-degree LiDAR scans at 10 Hz, range 0.12-20 m. |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo (via bridge) | (available for perception) | 640x480 RGB camera at 15 Hz. Used during dam scan task. |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo (via bridge) | (available for localisation) | IMU data at 50 Hz for orientation estimation. |
| `/battery_level_low` | `std_msgs/Bool` | `battery_simulator` | `CheckBatteryOK` (BT), `WaitForBatteryOK` (BT) | `true` when battery is below 20%, `false` once recharged past 90%. |
| `/is_docked` | `std_msgs/Bool` | `PublishDockStatus` (BT) | `battery_simulator` | Tells the battery simulator whether the robot is currently at the docking station, so it knows to charge instead of drain. |
| `/camera_controller/commands` | `std_msgs/Float64` | `SetCameraAngle` (BT) | Camera position controller | Commands the revolute camera joint to a specific yaw angle (in radians). |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo (via bridge) | All nodes (`use_sim_time: true`) | Simulation clock so all nodes use Gazebo's time. |

### Actions

| Action | Type | Server | Client | Description |
|--------|------|--------|--------|-------------|
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` | Nav2 `bt_navigator` | `NavigateToWaypoint` (BT node) | The primary navigation interface. |

**NavigateToPose action details:**

- **Goal**: A `geometry_msgs/PoseStamped` in the `map` frame containing the target x, y position and orientation (yaw converted to quaternion). For example, navigating to the survivor sends goal `(15.1, 11.9)` in the map frame.
- **Feedback**: Nav2 provides ongoing feedback including the robot's current pose and estimated time remaining (handled internally by Nav2; the BT node polls for completion via the result callback).
- **Result**: The BT node receives a `ResultCode`. If `SUCCEEDED`, the navigation is complete and the BT node returns `SUCCESS`. If the goal is rejected, failed, or cancelled, it returns `FAILURE`. The BT wraps each navigation call in a `Timeout` decorator (600 seconds) to prevent the robot from being stuck indefinitely.

### Services

No custom services were created for this project. The Nav2 stack internally uses lifecycle services for node management (e.g., `lifecycle_manager_navigation` transitions nodes through configure/activate states). The system communicates primarily through topics and actions.


## Section 2 - Behaviour Tree

### Overall Design

The behaviour tree is defined in `rescue_mission.xml` using BehaviorTree.CPP v4 format. It contains two trees: the main `RescueMission` tree and a `DockAndRecharge` subtree. The main tree is structured as a `ReactiveSequence` at the top level to enforce fire safety on every tick, with an inner `Sequence` that runs the mission tasks in order.

### Tree Structure

```
RescueMission (ReactiveSequence "fire_guard")
|
+-- CheckFireSafe                          [Condition - checked every tick]
|
+-- Sequence "mission"
    |
    +-- Sequence "task1_survivor_rescue"
    |   +-- ReactiveSequence
    |   |   +-- ReactiveFallback
    |   |   |   +-- CheckBatteryOK         [Condition]
    |   |   |   +-- SubTree: DockAndRecharge
    |   |   +-- Timeout (600s)
    |   |       +-- NavigateToWaypoint (survivor: 15.1, 11.9)
    |   +-- PublishDockStatus (false)
    |   +-- WaitSeconds (1.0s)
    |   +-- PublishObjectTF ("survivor_location": 15.1, 13.4)
    |   +-- ReactiveSequence               [battery-guarded navigation]
    |   |   +-- ReactiveFallback
    |   |   |   +-- CheckBatteryOK
    |   |   |   +-- SubTree: DockAndRecharge
    |   |   +-- Timeout (600s)
    |   |       +-- NavigateToWaypoint (medical_kit: -6.3, -18.4)
    |   +-- PublishDockStatus (false)
    |   +-- ReactiveSequence               [battery-guarded navigation]
    |   |   +-- ReactiveFallback
    |   |   |   +-- CheckBatteryOK
    |   |   |   +-- SubTree: DockAndRecharge
    |   |   +-- Timeout (600s)
    |   |       +-- NavigateToWaypoint (survivor_return: 15.1, 11.9)
    |   +-- PublishDockStatus (false)
    |
    +-- Sequence "task2_dam_scan"
    |   +-- ReactiveSequence               [battery-guarded navigation]
    |   |   +-- ... (same battery pattern)
    |   |   +-- NavigateToWaypoint (dam: 8.7, -13.1)
    |   +-- PublishDockStatus (false)
    |   +-- SetCameraAngle (0.0 rad)       [centre]
    |   +-- WaitSeconds (2.0s)
    |   +-- SetCameraAngle (0.1745 rad)    [10 degrees left]
    |   +-- WaitSeconds (2.0s)
    |   +-- SetCameraAngle (-0.1745 rad)   [10 degrees right]
    |   +-- WaitSeconds (2.0s)
    |   +-- SetCameraAngle (0.0 rad)       [back to centre]
    |
    +-- Sequence "task5_exit"
        +-- ReactiveSequence               [battery-guarded navigation]
        |   +-- ... (same battery pattern)
        |   +-- NavigateToWaypoint (exit: 2.9, 15.7)
        +-- PublishDockStatus (false)
        +-- WaitSeconds (5.0s)

DockAndRecharge (Sequence)
+-- NavigateToWaypoint (docking_station: 23.5, 0.0)
+-- PublishDockStatus (true)
+-- WaitForBatteryOK
```

### Node Types Used

**Sequence nodes:**
- The inner `"mission"` Sequence ensures Task 1, Task 2, and Task 5 run in order. Each task also has its own Sequence to chain the steps within that task (e.g., navigate, wait, publish TF, navigate again).

**Fallback nodes:**
- `ReactiveFallback` is used for battery handling. On every tick, it first checks `CheckBatteryOK`. If the battery is fine (returns `SUCCESS`), the fallback succeeds immediately and the navigation child proceeds. If the battery is low (returns `FAILURE`), the fallback ticks the `DockAndRecharge` subtree instead, which interrupts whatever navigation was happening and sends the robot to charge.

**Asynchronous stateful Action nodes:**
- `NavigateToWaypoint` - This is the main async action node. It sends a `NavigateToPose` goal to Nav2 in `onStart()`, returns `RUNNING` while Nav2 is executing, and checks the result callback in `onRunning()`. It properly implements `onHalted()` to cancel the Nav2 goal if the BT preempts it (e.g., due to low battery). If the goal is initially rejected (Nav2 not ready yet), it retries on the next tick.
- `WaitSeconds` - Records a target time in `onStart()` and returns `RUNNING` until the clock passes that time.
- `WaitForBatteryOK` - Subscribes to `/battery_level_low` and returns `RUNNING` until the battery is no longer low. Used during the docking sequence to keep the robot stationary until it is recharged.

**Decorator nodes:**
- `Timeout` (built-in BT.CPP decorator) wraps every `NavigateToWaypoint` call with a 600-second (10-minute) timeout. If navigation takes longer than this, the timeout decorator forces a `FAILURE`, preventing the robot from being stuck trying to reach an unreachable goal forever.

**Condition nodes:**
- `CheckBatteryOK` - Subscribes to `/battery_level_low` and returns `SUCCESS` if the battery is fine, `FAILURE` if low. Used in the `ReactiveFallback` pattern.
- `CheckFireSafe` - Looks up the robot's position via TF (`map -> base_link`) and calculates the Euclidean distance to the known fire location at (-14.2, 10.8). Returns `FAILURE` if the robot is closer than 3.0 m, which causes the top-level `ReactiveSequence` to halt all mission actions. If the TF lookup fails (e.g., during startup), it defaults to `SUCCESS` to avoid blocking the mission.

**Sync Action nodes:**
- `PublishObjectTF` - Publishes a static transform from `map` to a named child frame (e.g., `survivor_location`) at the specified coordinates. Used to report the survivor's location relative to spawn.
- `SetCameraAngle` - Publishes a `Float64` to the camera controller topic to set the revolute camera joint to a specific angle. Used for the dam scan sequence (centre, +10 degrees, -10 degrees).
- `PublishDockStatus` - Publishes a `Bool` to `/is_docked` to tell the battery simulator whether the robot is docked (charging) or not (draining).

### Ports (Inputs/Outputs)

| Node | Input Ports | Output Ports |
|------|------------|-------------|
| `NavigateToWaypoint` | `x` (double), `y` (double), `theta` (double, default 0.0), `description` (string) | None |
| `WaitSeconds` | `seconds` (double, default 1.0) | None |
| `PublishObjectTF` | `frame_name` (string), `x` (double), `y` (double) | None |
| `SetCameraAngle` | `angle` (double, default 0.0) - yaw in radians | None |
| `PublishDockStatus` | `docked` (bool) | None |
| `CheckBatteryOK` | None | None |
| `CheckFireSafe` | None | None |
| `WaitForBatteryOK` | None | None |

All coordinate values are set directly in the XML tree (e.g., `x="15.1" y="11.9"`). The BT nodes access the shared ROS node via the blackboard (key: `"node"`), which is set by the `mission_bt_node` main executable before the tree is created.


## Section 3 - Navigation Strategy

### Approach: Nav2 with SLAM Toolbox

The robot uses the **Nav2 navigation stack** combined with **SLAM Toolbox** for autonomous navigation. This was chosen because the robot needs to navigate a previously unknown environment (post-disaster building), so it builds a map on the fly rather than relying on a pre-made map.

### How It Works

1. **Mapping**: SLAM Toolbox runs in online asynchronous mode, consuming LiDAR scans from `/scan` and wheel odometry from `/diff_drive_controller/odom`. It produces a 2D occupancy grid map and publishes the `map -> odom` transform, giving the robot a global position estimate.

2. **Path planning**: When the BT sends a `NavigateToPose` goal, Nav2's **SmacPlanner2D** (lattice-based planner) computes a global path from the robot's current position to the target on the occupancy grid. The planner allows traversal through unknown space (`allow_unknown: true`) since the robot is exploring.

3. **Path following**: The **Regulated Pure Pursuit** controller tracks the planned path locally. It adjusts speed based on curvature (slows down around tight turns via `regulated_linear_scaling_min_radius`) and has a desired speed of 0.5 m/s. The controller outputs `Twist` commands on `/cmd_vel`.

4. **Velocity processing**: The velocity smoother smooths the commands from the controller, then `twist_relay` converts `Twist` to `TwistStamped` and forwards to `/diff_drive_controller/cmd_vel`.

5. **Obstacle avoidance**: Both local and global costmaps use an obstacle layer fed by LiDAR data and an inflation layer (0.55 m radius, robot radius 0.22 m). Nav2's recovery behaviours (spin, backup, wait) are configured to handle situations where the robot gets stuck.

### Navigation to Each Object

All target coordinates are hardcoded as waypoints in the behavior tree XML, based on the known positions of objects in the Gazebo world:

| Object | Coordinates (map frame) | Standoff | Notes |
|--------|------------------------|----------|-------|
| Survivor (green cuboid) | (15.1, 11.9) | ~1 m | Nav2 goal tolerance is 0.5 m; the waypoint is placed ~1 m from the actual object position. |
| Medical kit (yellow cube) | (-6.3, -18.4) | Close approach | Robot navigates to the kit and then returns to the survivor. |
| Dam (blue cube) | (8.7, -13.1) | ~0.5 m | After arriving, the camera joint pans to scan the dam. |
| Docking station (black square) | (23.5, 0.0) | Direct contact | Robot stops at the station and waits for recharge. |
| Exit (purple cube) | (2.9, 15.7) | Direct contact | Robot touches the exit marker and stops. |
| Fire (red cylinder) | (-14.2, 10.8) | 3.0 m minimum | Not a navigation target; this is the exclusion zone enforced by `CheckFireSafe`. |

### Safety Policies

**Fire avoidance (Task 4)**: The `CheckFireSafe` condition is placed at the top of the tree inside a `ReactiveSequence`. This means it is re-evaluated on *every single tick* of the tree (10 Hz). If the robot is ever closer than 3.0 m to the fire at (-14.2, 10.8), the entire mission halts. The distance is calculated using a TF lookup of the robot's base_link position in the map frame. In practice, Nav2's costmaps should route around the fire, but this condition acts as a hard safety check.

**Low battery (Task 3)**: Every navigation action is wrapped in a `ReactiveSequence` with a `ReactiveFallback` that checks battery status. If `/battery_level_low` becomes `true` mid-navigation, the `ReactiveFallback` triggers the `DockAndRecharge` subtree, which:
1. Cancels the current Nav2 goal (via `onHalted()`)
2. Navigates to the docking station at (23.5, 0.0)
3. Publishes `docked = true` on `/is_docked` so the battery simulator starts charging
4. Waits until the battery level is restored (above 90%)
5. The mission then resumes from where it left off

### Transform Chain

The navigation system relies on this transform chain:

```
map -> odom -> base_link -> [sensor frames]
```

- `map -> odom`: Published by SLAM Toolbox (corrects odometry drift)
- `odom -> base_link`: Published by `diff_drive_controller` (wheel odometry)
- `base_link -> lidar_link, camera_link, imu_link`: Published by `robot_state_publisher` (from URDF)


## Section 4 - References

### ROS 2 and Core Tools
- ROS 2 Jazzy documentation: https://docs.ros.org/en/jazzy/
- URDF tutorials: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html
- ros2_control documentation: https://control.ros.org/jazzy/
- `diff_drive_controller` documentation: https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
- `gz_ros2_control` integration: https://github.com/ros-controls/gz_ros2_control

### Gazebo
- Gazebo Harmonic documentation: https://gazebosim.org/docs/harmonic/
- `ros_gz_sim` and `ros_gz_bridge` packages: https://github.com/gazebosim/ros_gz
- Gazebo sensor plugins (LiDAR, camera, IMU): https://gazebosim.org/docs/harmonic/sensors

### Navigation
- Nav2 documentation: https://docs.nav2.org/
- Regulated Pure Pursuit controller: https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox

### Behaviour Trees
- BehaviorTree.CPP v4 documentation: https://www.behaviortree.dev/
- BehaviorTree.CPP GitHub: https://github.com/BehaviorTree/BehaviorTree.CPP
- Nav2 BT Navigator plugin: https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html

### Tutorials and Guides Used
- Articulated Robotics ROS 2 robot building series (URDF, Gazebo, ros2_control): https://articulatedrobotics.xyz/
- Nav2 getting started tutorials: https://docs.nav2.org/getting_started/index.html
- ROS 2 launch file tutorial: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html
- ME4857 Tutorials: https://learn.ul.ie/d2l/home/73043

