import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("search_rescue_robot")
    urdf_file = os.path.join(pkg_dir, "urdf", "robot.urdf")
    world_file = os.path.join(pkg_dir, "worlds", "assignment_world.sdf")

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    # Resolve $(find search_rescue_robot) — this is xacro syntax that doesn't
    # work in plain URDF. Replace it with the actual installed package path.
    robot_description = robot_description.replace(
        "$(find search_rescue_robot)", pkg_dir
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # --- Gazebo Harmonic ---
    gazebo = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", world_file,
        ],
        output="screen",
    )

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    # --- Spawn robot at (0, 0) ---
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "rescue_robot",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.1",
        ],
        output="screen",
    )

    # --- ros_gz_bridge for sensors, drive, and clock ---
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel_out@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true",
                description="Use Gazebo simulation clock",
            ),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            bridge,
        ]
    )
