from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("search_rescue_robot"))

    sim_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / "sim_spawn.launch.py"))
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / "mission.launch.py"))
    )

    return LaunchDescription([sim_spawn, mission])
