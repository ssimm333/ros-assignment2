from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    linear_speed = LaunchConfiguration("linear_speed")
    angular_speed = LaunchConfiguration("angular_speed")
    forward_duration = LaunchConfiguration("forward_duration")
    turn_duration = LaunchConfiguration("turn_duration")

    return LaunchDescription([
        DeclareLaunchArgument("linear_speed", default_value="0.2"),
        DeclareLaunchArgument("angular_speed", default_value="0.6"),
        DeclareLaunchArgument("forward_duration", default_value="3.0"),
        DeclareLaunchArgument("turn_duration", default_value="1.6"),
        Node(
            package="search_rescue_robot",
            executable="robot_controller",
            name="robot_controller",
            output="screen",
            parameters=[
                {"linear_speed": linear_speed},
                {"angular_speed": angular_speed},
                {"forward_duration": forward_duration},
                {"turn_duration": turn_duration},
            ],
        )
    ])
