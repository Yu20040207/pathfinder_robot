"""Localize against a saved real map with motors disabled by default."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os
import tempfile

import yaml
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fwr_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    localizer = LaunchConfiguration('localizer')
    amcl_selected = IfCondition(PythonExpression(["'", localizer, "' == 'amcl'"]))
    slam_selected = IfCondition(PythonExpression(["'", localizer, "' == 'slam'"]))
    # slam 节点由下面的 OpaqueFunction 启动：它需要一个“上次位姿”的运行时参数文件。
    start_slam = 'false'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e00aaf475431f111b7818c5f30d20014-if00-port0'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('map', default_value=str(share / 'maps' / 'real_map_merged.yaml')),
        # localizer:=amcl 用通用 AMCL + 地图图片；localizer:=slam 走 slam_toolbox
        # 的 localization 模式（加载建图时序列化的 posegraph，边定位边微调地图）。
        DeclareLaunchArgument('localizer', default_value='amcl'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=str(share / 'config' / 'real_slam_localization.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'real_mapping.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'lidar_port': LaunchConfiguration('lidar_port'),
                'start_imu': LaunchConfiguration('start_imu'),
                'start_slam': start_slam,
                'slam_params': LaunchConfiguration('slam_params'),
            }.items()),
        OpaqueFunction(function=_start_slam_localization,
                       args=[LaunchConfiguration('localizer')]),
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen', condition=amcl_selected,
             parameters=[str(share / 'config' / 'real_localization.yaml'),
                         {'yaml_filename': LaunchConfiguration('map'),
                          'use_sim_time': use_sim_time}]),
        Node(package='nav2_amcl', executable='amcl',
             name='amcl', output='screen', condition=amcl_selected,
             parameters=[str(share / 'config' / 'real_localization.yaml'),
                         {'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             condition=amcl_selected,
             parameters=[{'use_sim_time': use_sim_time,
                          'autostart': True,
                          'node_names': ['map_server', 'amcl']}]),
    ])


def _last_pose():
    """Read the checkpoint written by pose_checkpoint; None when unavailable."""
    path = os.path.expanduser('~/.config/fwr/last_pose.yaml')
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        return [float(data['x']), float(data['y']), float(data['yaw'])]
    except (OSError, KeyError, TypeError, ValueError):
        return None


def _start_slam_localization(context, localizer):
    from ament_index_python.packages import get_package_share_directory
    from launch_ros.actions import Node

    # args 传进来是 LaunchConfiguration 对象，需要显式求值。
    value = localizer if isinstance(localizer, str) else context.perform_substitution(localizer)
    print(f'[fwr_launch] localizer={value!r}', flush=True)
    if value != 'slam':
        return []
    share = Path(get_package_share_directory('fwr_bringup'))
    with open(share / 'config' / 'real_slam_localization.yaml', 'r', encoding='utf-8') as handle:
        params = yaml.safe_load(handle)
    pose = _last_pose()
    if pose is not None:
        params['slam_toolbox']['ros__parameters']['map_start_pose'] = pose
    handle, path = tempfile.mkstemp(prefix='fwr_slam_loc_', suffix='.yaml')
    with os.fdopen(handle, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(params, stream)
    import logging
    logging.getLogger('fwr_launch').info(
        'slam localization start pose %s', pose if pose else 'from config')
    return [Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                 name='slam_toolbox', output='screen', parameters=[path])]
