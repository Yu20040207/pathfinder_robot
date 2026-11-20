"""Real C1 + encoder + IMU mapping; base motion remains disabled by default."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fwr_bringup'))
    description = (share / 'description' / 'fwr_chassis_demo.urdf').read_text()
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e00aaf475431f111b7818c5f30d20014-if00-port0'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        # IMU 航向尺度修正：2026-10-30 雷达互相关标定（转一圈 355.04° vs 真值 359.55°）
        DeclareLaunchArgument('imu_yaw_scale', default_value='1.019'),  # 2026-10-30 雷达互相关实测
        DeclareLaunchArgument('start_slam', default_value='true'),
        # 2026-11-12：雷达可以由外部看门狗独占（start_lidar:=false），
        # 避免 launch 与看门狗同时开串口 → sllidar_node 死锁在 futex 上、/scan 断流。
        DeclareLaunchArgument('start_lidar', default_value='true'),
        # 2026-11-18：后雷达（第二只 C1）。start_rear_lidar:=false 可单独关掉，
        # 便于回退到单雷达方案做 A/B 对比。
        DeclareLaunchArgument('start_rear_lidar', default_value='true'),
        # 2026-11-18 用户要求：车尾改用后雷达，超声先不用（可 start_ultrasonic:=true 切回）
        DeclareLaunchArgument('start_ultrasonic', default_value='false'),
        DeclareLaunchArgument('lidar_rear_port', default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_14da82d39131f111a0eb945f30d20014-if00-port0'),
        DeclareLaunchArgument('slam_executable', default_value='async_slam_toolbox_node'),
        DeclareLaunchArgument('slam_map_topic', default_value='/map',
                              description='SLAM 地图话题（手机建图用 /slam_map 避免混入其它节点的 /map）'),
        DeclareLaunchArgument('slam_params',
                              default_value=str(share / 'config' / 'real_slam.yaml')),
        Node(package='fwr_bringup', executable='base_bridge', name='base_bridge',
             output='screen', parameters=[{'motion_enabled': False,
                                           'use_sim_time': use_sim_time}]),
        Node(package='fwr_imu', executable='dm_imu_node', name='dm_imu_l1',
             condition=IfCondition(LaunchConfiguration('start_imu')), output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          # 2026-10-30 实测：六轴航向少算 1.25%，乘 1.0127 修正
                          'yaw_scale': LaunchConfiguration('imu_yaw_scale')}]),
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             condition=IfCondition(LaunchConfiguration('start_lidar')),
             output='screen', respawn=True, respawn_delay=2.0,
             parameters=[{
                 'channel_type': 'serial', 'serial_port': LaunchConfiguration('lidar_port'),
                 'serial_baudrate': 460800, 'frame_id': 'laser_front',
                 'inverted': False, 'angle_compensate': True,
                 'scan_mode': 'Standard', 'use_sim_time': use_sim_time,
             }]),
        Node(package='fwr_bringup', executable='scan_mapping_filter',
             name='scan_mapping_filter', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          # 2026-11-18：遮罩 ±80°（**200° 视场**）——重叠从 80° 降到 40°，
                          # 重复点减半，同时保留 40° 冗余（单雷达故障时仍有 200° 覆盖）。
                          # 两路各 200°，前保 base_link ±100°，后保 80..280°，并集完整 360°。
                                                    'rear_half_angle_rad': math.radians(80.0)}]),

        # ---- 2026-11-18 双雷达：后雷达驱动 ----
        # 与车头那只同型号（C1），frame=laser_rear，话题 remap 到 /scan_rear，
        # 不占用 /scan（车头那路仍供单雷达回退使用）。
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node_rear',
             condition=IfCondition(LaunchConfiguration('start_rear_lidar')),
             output='screen', respawn=True, respawn_delay=2.0,
             parameters=[{
                 'channel_type': 'serial',
                 'serial_port': LaunchConfiguration('lidar_rear_port'),
                 'serial_baudrate': 460800, 'frame_id': 'laser_rear',
                 'inverted': False, 'angle_compensate': True,
                 'scan_mode': 'Standard', 'use_sim_time': use_sim_time,
             }],
             remappings=[('scan', '/scan_rear')]),
        # 第二路遮罩：同一节点换个实例，用绝对名 remap 到后雷达话题。
        # 遮罩同样 0°±70°（两只雷达的 scan 0 都指向车体内部）。
        Node(package='fwr_bringup', executable='scan_mapping_filter',
             name='scan_mapping_filter_rear',
             condition=IfCondition(LaunchConfiguration('start_rear_lidar')),
             output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'rear_half_angle_rad': math.radians(80.0)}],
             remappings=[('/scan', '/scan_rear'),
                         ('/scan_mapping', '/scan_rear_mapping'),
                         ('/scan_mapping/healthy', '/scan_rear_mapping/healthy')]),

        # ---- 双雷达合并：slam_toolbox 只吃单路，把两路 220° 合成 360° ----
        # （Cartographer 原生支持多路，不需要这个节点；只在使用 slam 引擎时有用）
        Node(package='fwr_bringup', executable='scan_merger', name='scan_merger',
             condition=IfCondition(LaunchConfiguration('start_rear_lidar')),
             output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'scan_a': '/scan_mapping',
                          'scan_b': '/scan_rear_mapping',
                          'output_topic': '/scan_merged',
                          'target_frame': 'base_link'}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             remappings=[('odometry/filtered', '/odom')],
             parameters=[str(share / 'config' / 'real_ekf.yaml')]),
        # 2026-11-19：slam_toolbox 延后 12 秒启动。
        # 原因：建图栈刚起时后雷达/scan_merger 还没出数据，SLAM 用"半圈"的
        # 首帧就把地图窗口定死了；车不动时 slam 不会重算，地图就永远缺一半。
        # 等双雷达+合并稳定后再开始吃扫描，首帧就是 360°。
        TimerAction(period=35.0, actions=[
        Node(package='slam_toolbox', executable=LaunchConfiguration('slam_executable'),
             name='slam_toolbox', output='screen',
             remappings=[('/map', LaunchConfiguration('slam_map_topic'))],
             condition=IfCondition(LaunchConfiguration('start_slam')),
             # 2026-11-18：双雷达时 slam_toolbox 2.6.10 只能吃一路，
             # 所以改用 scan_merger 合成的 /scan_merged（360°）；
             # 单雷达（start_rear_lidar:=false）仍用 /scan_mapping。
             parameters=[LaunchConfiguration('slam_params'),
                         {'scan_topic': PythonExpression([
                             "'/scan_merged' if '", LaunchConfiguration('start_rear_lidar'),
                             "' == 'true' else '/scan_mapping'"])}]),
        ]),
        # 给 RViz 看的降采样地图：slam_toolbox 的 /map 是 Reliable+TransientLocal、
        # 一帧 149 KB，RViz 在 WiFi 上做可靠消费者会触发重传风暴（实测放大 23 倍、
        # 遥控明显卡顿）。这路改成 1/2 降采样 + 0.5 Hz + Best Effort。
        Node(package='fwr_bringup', executable='map_view', name='map_view',
             condition=IfCondition(LaunchConfiguration('start_slam')),
             output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'in_topic': LaunchConfiguration('slam_map_topic'),
                          'out_topic': '/map_view',
                          # 2026-11-18 结论：**整图 + 50mm 全分辨率**是可行的。
                          # 关键不是裁小，而是不让 RViz 直接连 slam_toolbox 的 /map
                          # （那是 Reliable+TransientLocal，149 KB 一帧会在 WiFi 上
                          # 触发重传风暴，实测 7000+ KB/s）；改由本节点以
                          # Best Effort+Volatile、0.5 Hz 转发后，整图只要 ~230 KB/s。
                          # 不降采样（factor 1）是为了墙厚真实：降采样按"占用优先"
                          # 合并会把 50mm 图上的 100mm 墙显示成 200mm（用户反馈"墙变粗"）。
                          # crop_window_m=0 表示不裁剪，发整张地图。
                          'downsample_factor': 1,
                          'crop_window_m': 0.0,
                          'period_s': 2.0}]),
        # RViz「保存地图」按钮的服务：一次保存 .posegraph/.data + .pgm/.yaml。
        # 目录默认取 FWR_MAP_DIR（start_manual_mapping.sh 会设为源码 maps/），
        # 未设置时保存到 ~/robot_maps。
        Node(package='fwr_bringup', executable='slam_map_save',
             name='slam_map_save', output='screen',
             parameters=[{'map_topic': LaunchConfiguration('slam_map_topic')}]),
        # 后向四路 A22 超声（DYP-E08 / RS485）：发布 4 路 Range。
        # 2026-11-18：车尾已有后雷达（200° 视场、水平面测距、比超声可靠），
        # 默认不再启动超声；需要时 start_ultrasonic:=true 切回。
        Node(package='fwr_bringup', executable='ultrasonic_rear',
             name='ultrasonic_rear', output='screen',
             condition=IfCondition(LaunchConfiguration('start_ultrasonic')),
             parameters=[{'use_sim_time': use_sim_time}]),
        # 车尾守卫：近障时禁止后退（不产生任何前进/倒车指令）。
        # 源默认改成后雷达的正后方 ±35° 扇区；source:=ultrasonic 可切回超声。
        Node(package='fwr_bringup', executable='rear_guard',
             name='rear_guard', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'source': PythonExpression([
                              "'ultrasonic' if '",
                              LaunchConfiguration('start_ultrasonic'),
                              "' == 'true' else 'lidar'"]),
                          'scan_topic': '/scan_rear_mapping'}]),
        # 四只探头的静态 TF（安装见 docs/hardware/rear-a22-ultrasonic-layout.md）
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_ultrasonic_rear_left_out', output='log',
             arguments=['--x', '-0.26', '--y', '0.105', '--z', '0.14',
                        '--yaw', '3.316125578789226', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_footprint',
                        '--child-frame-id', 'ultrasonic_rear_left_out']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_ultrasonic_rear_left_in', output='log',
             arguments=['--x', '-0.26', '--y', '0.05', '--z', '0.14',
                        '--yaw', '3.141592653589793', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_footprint',
                        '--child-frame-id', 'ultrasonic_rear_left_in']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_ultrasonic_rear_right_in', output='log',
             arguments=['--x', '-0.26', '--y', '-0.05', '--z', '0.14',
                        '--yaw', '3.141592653589793', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_footprint',
                        '--child-frame-id', 'ultrasonic_rear_right_in']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_ultrasonic_rear_right_out', output='log',
             arguments=['--x', '-0.26', '--y', '-0.105', '--z', '0.14',
                        '--yaw', '2.9670597283903604', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_footprint',
                        '--child-frame-id', 'ultrasonic_rear_right_out']),
    ])
