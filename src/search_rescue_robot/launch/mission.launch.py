"""Launches the full rescue mission (nav2 + battery sim + BT controller)."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('search_rescue_robot')

    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # Nav2 stack (includes sim.launch.py)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    # Battery simulator
    battery_sim = Node(
        package='search_rescue_robot',
        executable='battery_simulator',
        name='battery_simulator',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 3. Mission BT node (delayed to let Nav2 fully start)
    mission_bt = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='rescue_bt',
                executable='mission_bt_node',
                name='mission_bt_node',
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz2'),
        nav2_launch,
        battery_sim,
        mission_bt,
    ])
