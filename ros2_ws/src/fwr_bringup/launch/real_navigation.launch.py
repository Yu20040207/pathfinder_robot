"""Real mapping or saved-map localization plus Nav2; motion stays gated."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = Path(get_package_share_directory('fwr_bringup'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_saved_map = LaunchConfiguration('use_saved_map')
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
        DeclareLaunchArgument('use_saved_map', default_value='false'),
        # 自动建图时不需要 frontier_explorer（它抢发 /explore/state=stopped 会解除授权）
        DeclareLaunchArgument('start_explorer', default_value='true'),
        # amcl（地图图片）或 slam（slam_toolbox localization + posegraph）
        DeclareLaunchArgument('localizer', default_value='amcl'),
        DeclareLaunchArgument('map', default_value=str(share / 'maps' / 'real_map_merged.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'real_mapping.launch.py')),
            condition=UnlessCondition(use_saved_map),
            launch_arguments={'use_sim_time': use_sim_time,
                              'lidar_port': LaunchConfiguration('lidar_port'),
                              'start_imu': LaunchConfiguration('start_imu')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'real_localization.launch.py')),
            condition=IfCondition(use_saved_map),
            launch_arguments={'use_sim_time': use_sim_time,
                              'lidar_port': LaunchConfiguration('lidar_port'),
                              'start_imu': LaunchConfiguration('start_imu'),
                              'map': LaunchConfiguration('map'),
                              'localizer': LaunchConfiguration('localizer')}.items()),
        # Navigation lifecycle stays inactive until map/TF/scan/drive gates are verified.
        Node(package='fwr_bringup', executable='frontier_explorer',
             name='frontier_explorer', output='screen',
             condition=IfCondition(LaunchConfiguration('start_explorer')),
             parameters=[str(share / 'config' / 'real_exploration.yaml'),
                         {'use_sim_time': use_sim_time, 'require_bridge_arm': True,
                          'static_map': use_saved_map}]),
        # 启动后自动做一次全局雷达匹配并播种 AMCL（手动挪车/重启后的 kidnapped 恢复）。
        # slam localization 模式下每 2 分钟把地图（pgm/yaml + posegraph）存盘。
        Node(package='fwr_bringup', executable='map_autosave', output='screen',
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration('localizer'), "' == 'slam'"])),
             parameters=[{'use_sim_time': use_sim_time,
                          'map_name': str(share / 'maps' / 'real_map_2026-10-28'),
                          'period_s': 120.0}]),
        # 等 map/scan/地图坐标 TF 都就绪后自动激活导航生命周期，省掉手动 STARTUP。
        Node(package='fwr_bringup', executable='auto_activate_nav', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        # 持续保存 map→base 位姿，下次启动 slam localization 时自动作为起点。
        Node(package='fwr_bringup', executable='pose_checkpoint', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        # 房间名以 RViz 文字标注显示；清空 maps/rooms.yaml 可关闭。
        Node(package='fwr_bringup', executable='room_labels', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'rooms_file': str(share / 'maps' / 'rooms.yaml')}]),
        Node(package='fwr_bringup', executable='auto_initial_pose', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'scan_topic': '/scan_mapping'}]),
        Node(package='fwr_bringup', executable='auto_command_gate',
             name='auto_command_gate', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='fwr_bringup', executable='directional_safety_selector',
             name='directional_safety_selector', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'scan_topic': '/scan'}]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor_forward', output='screen',
             parameters=[str(share / 'config' / 'real_collision_monitor_forward.yaml'),
                         {'use_sim_time': use_sim_time}]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor_reverse', output='screen',
             parameters=[str(share / 'config' / 'real_collision_monitor_reverse.yaml'),
                         {'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_collision', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'autostart': True,
                          'node_names': ['collision_monitor_forward', 'collision_monitor_reverse']}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': params,
                              'autostart': 'false',
                              'use_composition': 'False'}.items()),
    ])
