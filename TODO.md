# Search and Rescue Robot — Build Plan

> Each step must be verified before moving on.
> Future Claude sessions: read this file, check progress, pick up where it left off.

## Preferences
- always update README.md for commands to run the project in all launch file configurations
- always use URDF, never xacro
- never change code in assignment_world.sdf


## Architecture Overview

```
Packages:
  search_rescue_robot  (ament_python)  — launch files, perception node, battery simulator
  rescue_bt            (ament_cmake)   — BehaviorTree.CPP mission controller

Drive:       ros2_control + gz_ros2_control plugin + diff_drive_controller
Navigation:  Nav2 (SLAM Toolbox + planner + controller)
Mission BT:  BehaviorTree.CPP (C++) with custom action/condition nodes
Simulator:   Gazebo Harmonic
ROS distro:  Jazzy
```

## World Object Coordinates (from assignment_world.sdf)

| Object | Color | Position (x, y) | Meaning |
|--------|-------|-----------------|---------|
| green_cuboid | Green | (15.1, 13.4) | Survivor |
| blue_cube | Blue | (8.7, -11.6) | Dam |
| red_cylinder | Red | (-14.2, 10.8) | Fire/hazard |
| yellow_cube | Yellow | (-6.3, -16.9) | Medical kit |
| purple_cylinder | Purple | (2.9, 17.2) | Exit |
| black_square_east_wall | Black | (24.89, 0) | Docking station |
| Arena walls | Grey | +/-25 on each axis | 50x50m arena boundary |

---

## Step 0 — Review and Understand the URDF ✅
- [x] Read `src/search_rescue_robot/urdf/robot.urdf`
- [x] Verify link/joint structure matches RobotStructure.md
- [x] Understand: base_link, left/right wheels, caster, lidar, camera (revolute), imu
- [x] Note: URDF currently has Gazebo DiffDrive plugin — this will be REPLACED with ros2_control

### Success criteria
- You can describe every link, joint, and sensor in the URDF
- You understand the coordinate frame layout

---

## Step 1 — Package Scaffolding (search_rescue_robot) ✅
- [x] Create `src/search_rescue_robot/package.xml` (ament_python, with all deps)
- [x] Create `src/search_rescue_robot/setup.py` (data_files for urdf, worlds, config, launch)
- [x] Create `src/search_rescue_robot/setup.cfg`
- [x] Create `src/search_rescue_robot/resource/search_rescue_robot` (empty marker)
- [x] Create `src/search_rescue_robot/search_rescue_robot/__init__.py`
- [x] Create empty directories: `launch/`, `config/`
- [ ] `colcon build --packages-select search_rescue_robot` succeeds

### Dependencies needed in package.xml
```
rclpy, geometry_msgs, nav_msgs, std_msgs, std_srvs, sensor_msgs,
tf2_ros, launch, launch_ros, robot_state_publisher,
ros_gz_sim, ros_gz_bridge, nav2_bringup, nav2_msgs,
slam_toolbox, cv_bridge
```

### Success criteria
- `colcon build` passes with zero errors
- Package is installed to `install/` correctly

---

## Step 2 — URDF: Add ros2_control Hardware Interface ✅
- [x] Remove the existing `<gazebo>` DiffDrive plugin from robot.urdf
- [x] Add `<ros2_control>` tag with `gz_ros2_control/GazeboSimSystem` hardware interface
- [x] Define command/state interfaces for left_wheel_joint and right_wheel_joint
- [x] Add `<gazebo>` plugin for `gz_ros2_control::GazeboSimROS2ControlPlugin`
- [x] Keep all existing sensor `<gazebo>` blocks (lidar, camera, IMU)
- [x] Keep the JointStatePublisher plugin for camera joint TF

### Key references
- Tutorial PDF pages 93-98 (ros2_control in Gazebo)
- ros2_control docs for gz_ros2_control

### Success criteria
- URDF is valid XML
- ros2_control tags define velocity command interfaces for both wheel joints
- The gz_ros2_control Gazebo plugin references the correct controller YAML

---

## Step 3 — ros2_control Config (YAML) ✅
- [x] Create `config/ros2_control.yaml`
- [x] Configure `controller_manager` with update_rate
- [x] Configure `joint_state_broadcaster`
- [x] Configure `diff_drive_controller` (wheel names, separation, radius, odom settings)
- [ ] Optionally configure `camera_controller` (position controller for camera_joint) — deferred to later

### Key parameters for diff_drive_controller
```yaml
left_wheel_names: ["left_wheel_joint"]
right_wheel_names: ["right_wheel_joint"]
wheel_separation: 0.33
wheel_radius: 0.075
publish_odom: true
publish_odom_tf: true
```

### Success criteria
- YAML is valid and matches the URDF joint names exactly

---

## Step 4 — Simulation Launch File (Gazebo + Robot + Bridge) ✅
- [x] Create `launch/sim_launch.py`
- [x] Launch Gazebo Harmonic with the assignment world
- [x] Launch `robot_state_publisher` with URDF (config path substitution for plain URDF)
- [x] Spawn robot via `ros_gz_sim create` at (0, 0, 0.1)
- [x] Launch `ros_gz_bridge` for: /clock, /scan, /camera/image_raw, /imu/data
- [x] Spawn `controller_manager` (via gz_ros2_control in URDF)
- [x] Spawn `joint_state_broadcaster` controller (sequenced after entity spawn)
- [x] Spawn `diff_drive_controller` controller (sequenced after joint_state_broadcaster)

### Bridge topics and directions
```
/clock                @rosgraph_msgs/msg/Clock      [gz→ros
/scan                 @sensor_msgs/msg/LaserScan    [gz→ros
/camera/image_raw     @sensor_msgs/msg/Image        [gz→ros
/imu/data             @sensor_msgs/msg/Imu          [gz→ros
/cmd_vel              @geometry_msgs/msg/Twist       ]ros→gz  (or via ros2_control)
/odom                 @nav_msgs/msg/Odometry        [gz→ros  (or via ros2_control)
/tf                   @tf2_msgs/msg/TFMessage       [gz→ros
/joint_states         @sensor_msgs/msg/JointState   [gz→ros
```
Note: When using ros2_control, /cmd_vel and /odom may be handled by the diff_drive_controller
rather than the bridge. Check which approach works.

### Success criteria
- `ros2 launch search_rescue_robot sim_launch.py` starts Gazebo with the world
- Robot appears at origin
- `ros2 topic list` shows /clock, /scan, /cmd_vel, /odom, /tf, /joint_states
- `ros2 topic echo /scan` shows lidar data
- `ros2 topic echo /odom` shows odometry
- Robot drives when you publish to /cmd_vel:
  `ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.2}}}" --once`

---

## Step 5 — Teleop Test (Manual Driving) ✅
- [x] Verify robot responds to velocity commands via diff_drive_controller
- [x] Test with `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel`
- [x] Verify odometry updates as robot moves
- [x] Verify TF tree is complete: odom→base_link→sensors
- [x] Verify lidar scan data is reasonable in RViz2

### Success criteria
- Robot drives forward, backward, turns left/right via keyboard
- TF tree is connected (check with `ros2 run tf2_tools view_frames`)
- Odom position changes as robot moves

---

## Step 6 — Nav2 Config and Launch ✅ (pending runtime verification)
- [x] Create `config/nav2_params.yaml` with:
  - SLAM Toolbox params (online async, map frame, odom frame, scan topic)
  - Controller server (MPPI with DiffDrive motion model — Jazzy default)
  - Planner server (NavfnPlanner)
  - Behavior server (spin, backup, wait, drive_on_heading)
  - Local costmap (rolling window, obstacle + inflation layers)
  - Global costmap (static + obstacle + inflation layers)
  - BT navigator
  - Smoother server
- [x] Create `launch/nav2.launch.py` that:
  - Includes sim.launch.py
  - Launches SLAM Toolbox (async_slam_toolbox_node)
  - Launches Nav2 nodes individually (controller, planner, behavior, smoother, bt_navigator, waypoint_follower)
  - Launches lifecycle_manager_navigation to manage Nav2 nodes
  - Launches twist_relay (Twist→TwistStamped bridge for ros2_control)
  - Optionally launches RViz2 (use_rviz:=true)
- [x] Create `twist_relay.py` node (cmd_vel bridge between Nav2 and diff_drive_controller)
- [x] Update setup.py with twist_relay entry_point
- [x] Update README.md with nav2 launch commands

### Architecture note
Nav2 publishes `geometry_msgs/Twist` on `/cmd_vel`.
diff_drive_controller expects `geometry_msgs/TwistStamped` on `/diff_drive_controller/cmd_vel`.
The `twist_relay` node bridges this gap. All odom references point to `/diff_drive_controller/odom`.

### Success criteria
- `ros2 launch search_rescue_robot nav2.launch.py` starts everything
- In RViz2: click "Nav2 Goal", robot navigates autonomously
- Robot avoids walls
- SLAM map builds as robot drives
- Can send goal to known object coordinates and robot arrives

---

## Step 7 — Python Nodes: Battery Simulator
- [ ] Create `search_rescue_robot/battery_simulator.py`
- [ ] Publishes `Bool` on `/battery_level_low` at 1 Hz
- [ ] Drain rate parameter (% per second, default 0.5)
- [ ] Low threshold parameter (default 20%)
- [ ] Subscribes to `/is_docked` (Bool) — recharges when docked
- [ ] Full threshold parameter for hysteresis (default 90%)
- [ ] Add entry_point in setup.py

### Success criteria
- `ros2 run search_rescue_robot battery_simulator` runs
- `ros2 topic echo /battery_level_low` starts showing `true` after ~160 seconds (100%→20% at 0.5%/s)

---

## Step 8 — Python Nodes: Perception Node (Optional Enhancement)
- [ ] Create `search_rescue_robot/perception_node.py`
- [ ] Subscribe to `/camera/image_raw`
- [ ] HSV color filtering for green, blue, red, yellow, purple
- [ ] Publish detections as JSON on `/detected_objects`
- [ ] Add entry_point in setup.py

### Success criteria
- Node runs and publishes detections when camera sees colored objects
- Note: This is optional — the BT can work with known waypoint coordinates without perception

---

## Step 9 — Package Scaffolding (rescue_bt — C++ BT package)
- [ ] Create `src/rescue_bt/` directory
- [ ] Create `package.xml` (ament_cmake, deps: rclcpp, rclcpp_action, behaviortree_cpp, nav2_msgs, geometry_msgs, std_msgs, tf2_ros, tf2_geometry_msgs, ament_index_cpp)
- [ ] Create `CMakeLists.txt`
- [ ] Create `include/rescue_bt/bt_nodes.hpp`
- [ ] Create `src/bt_nodes.cpp` (empty implementations)
- [ ] Create `src/mission_bt_node.cpp` (main executor)
- [ ] Create `behavior_trees/` directory for XML
- [ ] `colcon build --packages-select rescue_bt` succeeds

### Success criteria
- Package builds with zero errors
- `mission_bt_node` executable is installed

---

## Step 10 — BT Node: NavigateToWaypoint (Async Action)
- [ ] Implement `NavigateToWaypoint` as `BT::StatefulActionNode`
- [ ] Sends `NavigateToPose` goal to Nav2 action server
- [ ] Input ports: x, y, theta, description
- [ ] Returns RUNNING until Nav2 completes
- [ ] Supports halt (cancels navigation goal)

### Assignment requirement satisfied
- "At least one asynchronous stateful Action node" (Part 5, BT Design)

### Success criteria
- Can navigate robot to a coordinate via BT XML
- Handles success and failure from Nav2

---

## Step 11 — BT Nodes: Supporting Actions
- [ ] `WaitSeconds` (StatefulActionNode) — waits N seconds, returns RUNNING then SUCCESS
- [ ] `PublishObjectTF` (SyncActionNode) — publishes static TF for named object at (x,y)
- [ ] `SetCameraAngle` (SyncActionNode) — publishes Float64 to camera controller topic
- [ ] `PublishDockStatus` (SyncActionNode) — publishes Bool to /is_docked

### Success criteria
- Each node compiles and can be used in a test BT XML

---

## Step 12 — BT Nodes: Conditions
- [ ] `CheckBatteryOK` (ConditionNode) — subscribes to /battery_level_low, returns SUCCESS if battery OK
- [ ] `CheckFireSafe` (ConditionNode) — looks up robot pose via TF, checks distance to fire > 3m

### Assignment requirements satisfied
- Fire safety constraint (Task 4, 5 marks)
- Battery monitoring (Task 3, 10 marks)

### Success criteria
- CheckBatteryOK returns FAILURE when battery_level_low is true
- CheckFireSafe returns FAILURE when robot is within 3m of (-14.2, 10.8)

---

## Step 13 — BT Nodes: WaitForBatteryOK
- [ ] `WaitForBatteryOK` (StatefulActionNode) — blocks (RUNNING) until battery_level_low is false

### Success criteria
- Node stays RUNNING while battery is low, returns SUCCESS when recharged

---

## Step 14 — Behavior Tree XML: Mission Structure
- [ ] Create `behavior_trees/rescue_mission.xml`
- [ ] Root: ReactiveSequence (fire safety always checked)
- [ ] Fire safety: CheckFireSafe condition at top level
- [ ] Battery handler: ReactiveFallback wrapping normal ops vs docking
- [ ] Task 1 subtree: Navigate to survivor → wait → publish TF → get med kit → return
- [ ] Task 2 subtree: Navigate to dam → camera scan (centre, left, right)
- [ ] Task 3 subtree: Navigate to dock → set docked → wait for recharge → unset docked
- [ ] Task 5 subtree: Navigate to exit → wait

### Assignment requirement: BT node types
- Sequence nodes: task pipelines (Task 1, 2, 3, 5)
- Fallback nodes: battery handler
- Async stateful action: NavigateToWaypoint
- Decorator: Timeout decorator on navigation actions (or use ReactiveSequence as reactive pattern)

### Success criteria
- XML is valid BTCPP_format="4"
- All referenced node types are registered in the factory
- Tree loads without errors

---

## Step 15 — Mission Executor (main node)
- [ ] Update `mission_bt_node.cpp` to register all BT nodes
- [ ] Load XML from installed share path
- [ ] Set up blackboard with ROS node handle
- [ ] Tick loop at 10 Hz with rclcpp::spin_some

### Success criteria
- `ros2 run rescue_bt mission_bt_node` loads the tree and starts ticking

---

## Step 16 — Full Mission Launch File
- [ ] Create `launch/mission_launch.py`
- [ ] Includes nav2_launch.py (which includes sim_launch.py)
- [ ] Launches battery_simulator
- [ ] Launches perception_node (optional)
- [ ] Launches mission_bt_node

### Success criteria
- `ros2 launch search_rescue_robot mission_launch.py` starts everything
- Robot begins executing mission tasks autonomously

---

## Step 17 — Integration Testing: Task by Task

### Task 1 — Medical Kit to Survivor (10 marks)
- [ ] Robot navigates to survivor (15.1, 13.4), stops ~1m away
- [ ] Waits 1 second
- [ ] Publishes TF "survivor_location" at (15.1, 13.4) relative to map
- [ ] Navigates to medical kit (-6.3, -16.9)
- [ ] Returns to survivor

### Task 2 — Dam Scan (10 marks)
- [ ] Robot navigates to dam (8.7, -11.6), stops ~0.5m away
- [ ] Camera aims at centre (0 rad)
- [ ] Camera pans 10 degrees left (0.1745 rad)
- [ ] Camera pans 10 degrees right (-0.1745 rad)
- [ ] Camera resets to centre

### Task 3 — Low Battery (10 marks)
- [ ] When /battery_level_low publishes true, robot preempts current task
- [ ] Robot navigates to docking station (24.89, 0)
- [ ] Robot waits until battery is recharged
- [ ] Robot resumes mission

### Task 4 — Fire Safety (5 marks)
- [ ] Robot never goes within 3m of fire at (-14.2, 10.8)
- [ ] If too close, BT triggers retreat/avoidance

### Task 5 — Exit Building (5 marks)
- [ ] After all tasks complete, robot navigates to exit (2.9, 17.2)
- [ ] Robot stops at exit and waits

### Navigation (10 marks)
- [ ] Robot autonomously reaches all waypoints via Nav2
- [ ] Avoids obstacles (walls, objects)

### Success criteria
- Full mission runs from spawn to exit without manual intervention
- All tasks complete in correct order
- Battery docking works when triggered

---

## Step 18 — Report
- [ ] Fill in REPORT_TEMPLATE.md
- [ ] Section 1: ROS Architecture (nodes, topics, services, actions)
- [ ] Section 2: Behavior Tree design (node types, ports, tree structure)
- [ ] Section 3: Navigation strategy (Nav2, SLAM, planner choice)
- [ ] Section 4: References (all tutorials, docs, repos used)

---

## Step 19 — Final Cleanup and Commit
- [ ] `colcon build` succeeds with zero warnings
- [ ] All launch files work
- [ ] .gitignore covers build/, install/, log/, __pycache__/
- [ ] README.md is updated with build/run instructions
- [ ] Git commit with clean history
