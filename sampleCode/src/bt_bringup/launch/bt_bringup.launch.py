from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # BT Runner Node
        Node(
            package='behaviour_tree',
            executable='bt_runner',
            name='bt_runner',
            output='screen'
        ),
        
        # Drive X Forward Action Server
        Node(
            package='bt_action_server',
            executable='drive_x_forward_server',
            name='drive_x_forward_server',
            output='screen'
        ),
        
        # Rotate X Action Server
        Node(
            package='bt_action_server',
            executable='rotate_x_server',
            name='rotate_x_server',
            output='screen'
        ),
    ])
