import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("search_rescue_robot")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Include nav2 launch (which includes sim_launch)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "nav2_launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    perception = Node(
        package="search_rescue_robot",
        executable="perception_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    battery = Node(
        package="search_rescue_robot",
        executable="battery_simulator",
        parameters=[{"use_sim_time": use_sim_time, "drain_rate": 0.5}],
        output="screen",
    )

    mission_bt = Node(
        package="rescue_bt",
        executable="mission_bt_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        nav2_launch,
        perception,
        battery,
        mission_bt,
    ])
