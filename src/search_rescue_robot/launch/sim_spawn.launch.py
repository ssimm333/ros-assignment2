from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("search_rescue_robot"))
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    urdf_file = package_share / "urdf" / "robot.urdf"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=str(package_share / "worlds" / "assignment_world.sdf"),
                description="Path to Gazebo Harmonic world",
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.2"),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", world],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": urdf_file.read_text(encoding="utf-8")}, {"use_sim_time": True}],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name",
                    "rescue_rover_a2",
                    "-file",
                    str(urdf_file),
                    "-x",
                    x,
                    "-y",
                    y,
                    "-z",
                    z,
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["camera_pan_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ]
    )
