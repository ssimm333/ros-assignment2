# Search and Rescue Robot Report

## Section 1 - ROS Architecture

### Node overview
- `mission_controller` / `mission_bt_node`: BehaviorTree.CPP mission executor (C++), handles mission sequencing, battery handling, fire-safety checks, and publishing survivor TFs.
- `robot_state_publisher`: publishes the TF tree from the URDF.
- `ros_gz_sim` / spawn entity: spawns the robot in Gazebo Harmonic.
- `ros_gz_bridge` / `ros_gz_bridge parameter_bridge`: bridges Gazebo topics (/clock, sensors) to ROS.
- `controller_manager` + controllers: `joint_state_broadcaster`, `diff_drive_controller`, (optional) camera position controller.
- `battery_simulator` (Python): publishes `/battery_level_low` and handles recharge while docked.
- `twist_relay` (Python): translates Nav2 `Twist` → `TwistStamped` for `diff_drive_controller`.

Packages: `search_rescue_robot` (ament_python) — launch files, battery simulator, twist_relay, perception; `rescue_bt` (ament_cmake) — BehaviorTree.CPP mission controller.

### Topics and message types
- `/diff_drive_controller/cmd_vel` (geometry_msgs/TwistStamped) — accepts velocity commands for `diff_drive_controller`.
- `/cmd_vel` (geometry_msgs/Twist) — Nav2 output, relayed by `twist_relay` when used.
- `/battery_level_low` (std_msgs/Bool) — battery state published by `battery_simulator`.
- `/mission/events` (std_msgs/String) — mission status/events.
- `/camera_pan_controller/commands` (std_msgs/Float64MultiArray) — camera pan commands (controller topic).
- `/diff_drive_controller/odom` (nav_msgs/Odometry) — odometry from controller.
- `/scan`, `/camera/image_raw`, `/imu/data` (sensor_msgs) — Gazebo sensor topics bridged into ROS.

### Services and actions
- Services: example lifecycle or status services such as `/mission_controller/get_mission_status` (std_srvs/Trigger) used for test/telemetry.
- Actions: Nav2 `NavigateToPose` is used by BT action nodes (wrapped by `NavigateToWaypoint` async action). Custom waypoint action semantics are implemented in the BT nodes (goal: map-frame pose; feedback: progress; result: success/failure/timeouts).

## Section 2 - Behaviour Tree

### Tree shape (high level)
- Root: Reactive/Sequence that ensures mission steps proceed in order but allows reactive preemption for safety/battery.
- Top-level Fallback for fire-safety: `CheckFireSafe` condition → `Retreat` action.
- Battery handler: ReactiveFallback that either runs mission subtrees or preempts to dock and recharge.
- Task 1 Sequence: Navigate to survivor → Wait/publish TF → Navigate to medical kit → Return to survivor.
- Task 2 Sequence: Navigate to dam → Camera scan sequence (centre → left → right) → reset camera.
- Task 5 Sequence: Navigate to exit → Wait.

### Node types and roles
- Sequence: orchestrates ordered steps within tasks.
- Fallback: battery and fire-safety handlers to choose safe alternatives.
- Async stateful Action: `NavigateToWaypoint` (sends Nav2 goals, returns RUNNING while Nav2 executes, supports halt/cancel).
- Decorators: Timeout on navigation actions to bound retries.
- Conditions: `CheckBatteryOK` (subscribes to `/battery_level_low`), `CheckFireSafe` (TF lookup + distance check to fire coordinate).

### Ports and data flow
- Inputs: target coordinates (from world constants), battery status, odometry/TF for safety checks.
- Outputs: commands to `diff_drive_controller` (via Nav2), camera controller commands, published TF names for detected survivors.

## Section 3 - Navigation Strategy

- Primary navigation: Nav2 stack (used with SLAM Toolbox for mapping in simulation), planner = Navfn (global), controller = MPPI (local) as configured in `nav2_params.yaml`.
- SLAM: SLAM Toolbox (online async) for map building in sim when required.
- Twist bridging: Nav2 publishes `geometry_msgs/Twist`; `twist_relay` converts to `TwistStamped` and forwards to `/diff_drive_controller/cmd_vel`.

- Standoff / task distances:
  - Survivor: stop ≈ 1.0 m (publish TF for survivor location from this pose)
  - Dam: stop ≈ 0.5 m (camera scans with revolute joint ±10°)

- Safety policies:
  - Fire safety: `CheckFireSafe` enforces a 3.0 m exclusion zone around the fire marker (world coordinate: -14.2, 10.8). If violated, the BT triggers retreat/avoidance.
  - Low battery: when `/battery_level_low` is true (threshold default 20%), mission preempts and the robot navigates to the docking station (24.89, 0) and waits until recharge (hysteresis up to 90%).

- Controller and odometry notes:
  - `diff_drive_controller` provides odometry on `/diff_drive_controller/odom` and expects `TwistStamped` commands.
  - Odometry + IMU are fused (recommended) for robust localisation; Nav2 uses the odom→base_link→map transform chain.

## Section 4 - References

- ROS 2 documentation (Jazzy)
- Nav2 (Navigation2) documentation and MPPI controller docs
- Gazebo / Gazebo Harmonic (ros_gz_sim) and `ros_gz_bridge`
- ros2_control and `gz_ros2_control` integration guides
- BehaviorTree.CPP documentation and examples
- SLAM Toolbox documentation
