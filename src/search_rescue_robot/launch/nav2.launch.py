# nav2.launch.py
# Top-level launch that brings up the simulation, SLAM, Nav2 navigation stack,
# the twist relay, and optionally RViz for debugging.
# This is included by mission.launch.py which adds the battery sim and BT node.

"""Launches simulation + Nav2 + SLAM + twist relay."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('search_rescue_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # bring up the base simulation (Gazebo + robot + ros2_control)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments=[('use_sim_time', use_sim_time)],
    )

    # launch Nav2 and SLAM together as a group.
    # SLAM builds the map from lidar as the robot moves,
    # Nav2 uses that map for path planning and obstacle avoidance.
    nav2_group = GroupAction([
        # SLAM Toolbox: builds a 2D occupancy grid map from /scan
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'slam_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_respawn': 'False',
                'params_file': nav2_params_file,
            }.items(),
        ),

        # Nav2 navigation stack (planner, controller, behavior server, etc.)
        # using non-composition mode (each server is a separate process)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': nav2_params_file,
                'use_composition': 'False',
                'use_respawn': 'False',
            }.items(),
        ),
    ])

    # twist relay converts Nav2's Twist messages to TwistStamped
    # because the diff_drive_controller needs stamped velocity commands
    twist_relay = Node(
        package='search_rescue_robot',
        executable='twist_relay',
        name='twist_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RViz2 for visualising the map, robot pose, and nav paths (optional)
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz2 with Nav2 default view',
        ),
        sim_launch,
        nav2_group,
        twist_relay,
        rviz2,
    ])
