"""One-command navigation: saved-map localization + Nav2 + RViz.

This is a thin wrapper around the existing, already-validated
real_navigation.launch.py.  It does not change navigation parameters, the
safety chain, or the mapping stack.

Usage:
    ros2 launch fwr_bringup run_navigation.launch.py
    ros2 launch fwr_bringup run_navigation.launch.py \
        map:=/abs/path/to/real_map.yaml start_rviz:=false

Without map:=, the launch uses the same last-map state file as
scripts/run_navigation.sh (~/.config/fwr/last_navigation_map), so a direct
ros2 launch also reopens the previous map.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _last_map(share: Path) -> str:
    state = Path(os.environ.get(
        'FWR_NAV_LAST_MAP',
        str(Path.home() / '.config/fwr/last_navigation_map'))).expanduser()
    try:
        candidate = state.read_text(encoding='utf-8').splitlines()[0].strip()
        if candidate and Path(candidate).is_file():
            return candidate
    except (OSError, IndexError):
        pass
    return str(share / 'maps' / 'real_map_auto_2026-11-04_1525.yaml')


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('fwr_bringup'))
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')

    dds_profile = Path.home() / '.config/fwr/fastdds_udp_only.xml'

    return LaunchDescription([
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            str(dds_profile)),
        DeclareLaunchArgument(
            'map',
            default_value=_last_map(share),
            description='Saved map yaml to localize against.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e00aaf475431f111b7818c5f30d20014-if00-port0'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('localizer', default_value='amcl'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / 'launch' / 'real_navigation.launch.py')),
            launch_arguments={
                'use_saved_map': 'true',
                'use_sim_time': use_sim_time,
                'lidar_port': LaunchConfiguration('lidar_port'),
                'start_imu': LaunchConfiguration('start_imu'),
                'map': map_file,
                'localizer': LaunchConfiguration('localizer'),
                'start_explorer': 'true',
            }.items()),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', str(share / 'config' / 'real_localization.rviz')],
            condition=IfCondition(start_rviz),
            output='log'),
    ])
