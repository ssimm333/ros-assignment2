from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("search_rescue_robot"))
    robot_urdf = package_share / "urdf" / "robot.urdf"

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = robot_urdf.read_text(encoding="utf-8")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if available",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="search_rescue_robot",
                executable="mission_controller",
                name="mission_controller",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
