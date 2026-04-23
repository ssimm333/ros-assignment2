## Robot Structure

```
base_link (box)
|-- left_wheel (cylinder, continuous joint)
|-- right_wheel (cylinder, continuous joint)
|-- caster_wheel (sphere, fixed joint)
|-- lidar_link (cylinder, fixed joint)
|-- camera_link (box, revolute joint)
|-- imu_link (no visual, fixed joint)
```

### Layout

Differential drive. Two powered wheels, one caster at the back. Simple, works out of the box with `cmd_vel` and Nav2.

### base_link

A 0.4 x 0.3 x 0.15m box. Kept it small so it can get close to objects — the spec wants us within 50cm of the dam and 1m of the survivor. 5kg mass.

### Wheels

Cylinders, 0.075m radius. Continuous joints so they spin freely. Had to rotate them -90 degrees on the roll axis because URDF cylinders point up by default and we need them on their side. Placed just outside the body on each side.

### Caster

Small sphere at the back. Fixed joint, near-zero friction in the Gazebo config. Stops the robot tipping backwards.

### LiDAR

Sits on top, centre of the base. Full 360 degree view, nothing blocking it. Feeds into SLAM for navigation.

### Camera

Mounted on the front with a **revolute joint**. This is specifically for Task 2 — we need to pan the camera ±10 degrees to scan the dam. A revolute joint means the camera can sweep without the whole robot turning. Limits set a bit wider than 10 degrees to give some room.

### IMU

No visual, just a sensor frame inside the base. Helps `robot_localization` fuse odometry and orientation data for better position tracking.

### Inertials

Our links needs mass and an inertia matrix or else Gazebo will break. We used the standard box/cylinder/sphere formulas, to be approximate.

