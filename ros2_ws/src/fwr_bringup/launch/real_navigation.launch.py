"""Real mapping plus Nav2. Base motion and Nav2 lifecycle stay gated."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = Path(get_package_share_directory('fwr_bringup'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = RewrittenYaml(
        source_file=str(share / 'config' / 'real_nav2.yaml'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': str(share / 'behavior_trees' / 'real_navigate_to_pose.xml'),
            'default_nav_through_poses_bt_xml': str(share / 'behavior_trees' / 'real_navigate_through_poses.xml'),
        },
        convert_types=True,
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e00aaf475431f111b7818c5f30d20014-if00-port0'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'real_mapping.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'lidar_port': LaunchConfiguration('lidar_port'),
                              'start_imu': LaunchConfiguration('start_imu')}.items()),
        # Navigation lifecycle stays inactive until map/TF/scan/drive gates are verified.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': params,
                              'autostart': 'false',
                              'use_composition': 'False'}.items()),
    ])
