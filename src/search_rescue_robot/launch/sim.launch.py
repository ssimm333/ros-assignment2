# sim.launch.py
# Launches the Gazebo simulation with the robot spawned in the assignment world.
# Also starts ros2_control (joint state broadcaster + diff drive controller)
# and the ros_gz_bridge to forward sensor data from Gazebo into ROS2 topics.

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

    # paths to the robot model, world file, and controller config
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'assignment_world.sdf')
    controller_yaml = os.path.join(pkg_share, 'config', 'ros2_control.yaml')

    # read the URDF and replace the {CONFIG_PATH} placeholder with the
    # actual path to ros2_control.yaml. this is needed because the URDF
    # references the controller config but doesnt know where its installed.
    with open(urdf_file, 'r') as f:
        robot_description_raw = f.read()
    robot_description = robot_description_raw.replace('{CONFIG_PATH}', controller_yaml)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # launch Gazebo Harmonic with the assignment world
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

    # robot state publisher: reads the URDF and publishes the robot's
    # joint states and transforms so other nodes know where each link is
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

    # spawn the robot model into the running Gazebo simulation
    # it reads the URDF from the robot_description topic
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

    # ros2_control controller spawners.
    # joint_state_broadcaster reads joint positions from the simulation
    # and publishes them on /joint_states for robot_state_publisher to use.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # diff_drive_controller takes velocity commands and drives the wheels
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    # ros_gz_bridge forwards messages between Gazebo and ROS2.
    # each argument maps a Gazebo topic to a ROS topic with the right message types.
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

    # event handlers to sequence the startup correctly:
    # the controllers cant start until the robot is spawned in Gazebo,
    # and the diff drive cant start until joint_state_broadcaster is running.
    # OnProcessExit waits for the previous process to finish before starting the next.
    spawn_jsb_after_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    # once joint_state_broadcaster is up, start the diff drive controller
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
