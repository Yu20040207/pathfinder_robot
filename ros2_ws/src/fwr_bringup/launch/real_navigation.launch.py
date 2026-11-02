"""Real mapping or saved-map localization plus Nav2; motion stays gated."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
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
        # 2026-11-02：用"官方结构、去掉 BackUp"的 BT。
        # - 空字符串 ≠ 默认，那是**空树**（目标 2 ms 失败）
        # - 直接用官方 XML 会因缺 /backup 动作导致 BT 加载失败
        param_rewrites={
            'default_nav_to_pose_bt_xml': str(share / 'behavior_trees' /
                                              'real_navigate_to_pose.xml'),
        },
        convert_types=True,
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e00aaf475431f111b7818c5f30d20014-if00-port0'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('use_saved_map', default_value='false'),
        # 2026-10-30：建图引擎可选 slam（slam_toolbox）或 carto（Cartographer）。
        # 离线 bag 回放对比结论：Cartographer 墙更实（87.7% vs 84%）、方向更一致（6° vs 13.5°）。
        DeclareLaunchArgument('slam_engine', default_value='slam'),
        # 2026-10-31：加载已有 pbstream 继续建图（空 = 从头建）
        DeclareLaunchArgument('load_state', default_value=''),
        # 自动建图时不需要 frontier_explorer（它抢发 /explore/state=stopped 会解除授权）
        DeclareLaunchArgument('start_explorer', default_value='true'),
        # amcl（地图图片）或 slam（slam_toolbox localization + posegraph）
        DeclareLaunchArgument('localizer', default_value='amcl'),
        DeclareLaunchArgument('map', default_value=str(share / 'maps' / 'real_map_merged.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'real_mapping.launch.py')),
            condition=IfCondition(PythonExpression(
                ["'", use_saved_map, "' != 'true' and '",
                 LaunchConfiguration('slam_engine'), "' != 'carto'"])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'lidar_port': LaunchConfiguration('lidar_port'),
                              'start_imu': LaunchConfiguration('start_imu')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / 'launch' / 'real_mapping_cartographer.launch.py')),
            condition=IfCondition(PythonExpression(
                ["'", use_saved_map, "' != 'true' and '",
                 LaunchConfiguration('slam_engine'), "' == 'carto'"])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'lidar_port': LaunchConfiguration('lidar_port'),
                              'start_imu': LaunchConfiguration('start_imu'),
                              'load_state': LaunchConfiguration('load_state')}.items()),
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
        # 地图缝合：把 5~15 cm 的墙缝膨胀封住，供全局代价地图使用。
        # 2026-10-31 回归修复：原来只在建图模式启动，而静态层固定读 /map_sealed，
        # 导致导航模式下代价地图拿不到地图（只剩 5×5 m 滚动窗口，目标稍远就规划失败）。
        # 现在两种模式都启动这个中继。
        # 固定画布，与 real_nav2.yaml 全局代价地图的 width/height/origin 严格一致
        # （否则 StaticLayer 会跟着 /map_sealed 尺寸反复 Resizing → Smac 崩溃）
        Node(package='fwr_bringup', executable='map_seal', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          # 2026-11-01：固定画布搬移有 bug（实测密封图与真实地图
                          # 0/201 对不上，导致代价地图里的墙在错误位置 → 规划穿墙）。
                          # 先改回直通：尺寸随地图，配合代价地图的固定窗口由静态层
                          # 自行对齐（静态层用地图自带 origin，天然正确）。
                          'fixed_width_m': 0.0,
                          'fixed_height_m': 0.0,
                          # 2026-11-01：手动地图有噪点和缝隙 → 先清噪点，再膨胀 2 格封缝
                          'min_cluster_cells': 4,
                          'dilate_cells': 1}]),
        # 启动后自动做一次全局雷达匹配并播种 AMCL（手动挪车/重启后的 kidnapped 恢复）。
        # slam localization 模式下每 2 分钟把地图（pgm/yaml + posegraph）存盘。
        # 2026-10-30：自动存档在建图模式下也要跑（原来只在 slam localization 模式），
        # 引擎名一起传下去：slam → /serialize_map + /save_map；carto → /write_state。
        Node(package='fwr_bringup', executable='map_autosave', output='screen',
             condition=IfCondition(PythonExpression(
                 ["'", use_saved_map, "' != 'true' or '",
                  LaunchConfiguration('localizer'), "' == 'slam'"])),
             parameters=[{'use_sim_time': use_sim_time,
                          'engine': LaunchConfiguration('slam_engine'),
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
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_collision', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'autostart': True,
                          'node_names': ['collision_monitor_forward']}]),
        # 2026-11-02 回退：不再常驻启动 auto_explore_state。
        # 它无条件每秒发 /explore/state='navigating'，与 explorer 自己的状态机
        # 打架 —— 发出去的目标 0.1 s 就被 "Client requested to cancel the goal"，
        # 表现为"一开始就结束"。状态仍由 explorer 在导航期间自己发布。
        # 2026-11-02：看门狗**不再放进导航栈**。
        # 实测它把 collision_monitor_forward 误判为 unknown，强行 deactivate->activate
        # 失败后监控进入坏状态，/cmd_vel_auto_safe 恒为 0、车完全不动 —— 比它要修的问题更糟。
        # 需要时手动起：python3 tools/chain_watchdog.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': params,
                              'autostart': 'false',
                              'use_composition': 'False'}.items()),
    ])
