import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("search_rescue_robot")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    params_file = os.path.join(pkg_dir, "config", "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Include the simulation launch (Gazebo + robot + controllers + bridge)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "sim_launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # SLAM Toolbox — online async mapping
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Nav2 bringup (planner, controller, BT navigator, behaviors, costmaps)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": params_file,
            "enable_docking": "false",
        }.items(),
    )

    # RViz with Nav2 default view
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true",
            ),
            sim_launch,
            slam_toolbox,
            nav2_bringup,
            rviz,
        ]
    )
