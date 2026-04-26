# Search and Rescue Robot Report

## Section 1 - ROS Architecture

### Node overview
- mission_controller: Behavior Tree mission execution, battery handling, fire safety checks, survivor TF publication.
- robot_state_publisher: publishes robot TF tree from URDF.
- ros_gz_sim create: spawns robot in Gazebo Harmonic.
- ros_gz_bridge parameter_bridge: bridges simulation clock to ROS.
- controller_manager + controllers: drive and camera control interfaces.

### Topics and message types
- /cmd_vel (geometry_msgs/Twist)
- /battery_level_low (std_msgs/Bool)
- /mission/events (std_msgs/String)
- /camera_pan_controller/commands (std_msgs/Float64MultiArray)
- /odom (nav_msgs/Odometry)
- /scan, /camera/image_raw, /imu/data (from Gazebo sensors)

### Services used
- /mission_controller/get_mission_status (std_srvs/Trigger)

### Actions used
- Custom waypoint navigation action implemented inside mission_controller

#### Action details
- Goal: map-frame target pose (x, y, yaw).
- Feedback: internal state updates from odometry and velocity control.
- Result: success when waypoint is reached, failure on timeout or docking interruptions.

## Section 2 - Behaviour Tree

### Tree shape
- Root Sequence
- Fire safety Fallback
- Task 1 Sequence (survivor + med kit)
- Task 2 Sequence (dam scan)
- Task 5 Sequence (exit)

### Required node types satisfied
- Sequence: root and task pipelines.
- Fallback: fire safety fallback (safe check vs retreat action).
- Asynchronous stateful action: custom waypoint navigation node.
- Decorator: timeout decorator around navigation actions.

### Ports and data
- Inputs from battery topic and odometry topic.
- Target coordinates from world constants.
- Outputs to mission events topic and survivor TF broadcaster.

## Section 3 - Navigation Strategy

- Primary strategy: custom waypoint navigation using odometry and velocity control.
- Standoff logic:
  - survivor stop distance: 1.0 m
  - dam stop distance: 0.5 m
- Fire safety constraint: if robot is closer than 3.0 m to fire marker, mission requests retreat.
- Low battery behavior: mission preempts and docks at black docking marker until battery recovers.

## Section 4 - References

Add every source you used:
- ROS 2 docs
- Nav2 docs
- Gazebo Harmonic docs
- ros2_control docs
- Any tutorial links
- Any AI/chat assistance references
