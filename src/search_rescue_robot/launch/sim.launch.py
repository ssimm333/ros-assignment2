import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('search_rescue_robot')

    # --- Paths ---
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'assignment_world.sdf')
    controller_yaml = os.path.join(pkg_share, 'config', 'ros2_control.yaml')

    # --- Read URDF and substitute config path placeholder ---
    with open(urdf_file, 'r') as f:
        robot_description_raw = f.read()
    robot_description = robot_description_raw.replace('{CONFIG_PATH}', controller_yaml)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Launch Gazebo Harmonic ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ]),
        launch_arguments=[('gz_args', ['-r -v 1 ', world_file])],
    )

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # --- Spawn robot in Gazebo ---
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rescue_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-allow_renaming', 'true',
        ],
    )

    # --- Spawn controllers (sequenced after spawn) ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    # --- ros_gz_bridge: sensor topics only (drive handled by ros2_control) ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
    )

    # --- Event handlers: sequence controller spawning after entity spawn ---
    # Wait for entity to spawn, then start joint_state_broadcaster
    spawn_jsb_after_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    # Wait for joint_state_broadcaster, then start diff_drive_controller
    spawn_ddc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true',
        ),
        gz_sim,
        robot_state_publisher,
        gz_spawn_entity,
        bridge,
        spawn_jsb_after_entity,
        spawn_ddc_after_jsb,
    ])
