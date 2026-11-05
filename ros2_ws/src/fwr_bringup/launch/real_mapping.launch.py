"""Real C1 + encoder + IMU mapping; base motion remains disabled by default."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
        DeclareLaunchArgument('slam_executable', default_value='async_slam_toolbox_node'),
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
             output='screen', parameters=[{
                 'channel_type': 'serial', 'serial_port': LaunchConfiguration('lidar_port'),
                 'serial_baudrate': 460800, 'frame_id': 'laser_front',
                 'inverted': False, 'angle_compensate': True,
                 'scan_mode': 'Standard', 'use_sim_time': use_sim_time,
             }]),
        Node(package='fwr_bringup', executable='scan_mapping_filter',
             name='scan_mapping_filter', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'rear_half_angle_rad': math.radians(50.0)}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             remappings=[('odometry/filtered', '/odom')],
             parameters=[str(share / 'config' / 'real_ekf.yaml')]),
        Node(package='slam_toolbox', executable=LaunchConfiguration('slam_executable'),
             name='slam_toolbox', output='screen',
             condition=IfCondition(LaunchConfiguration('start_slam')),
             parameters=[LaunchConfiguration('slam_params')]),
        # RViz「保存地图」按钮的服务：一次保存 .posegraph/.data + .pgm/.yaml。
        # 目录默认取 FWR_MAP_DIR（start_manual_mapping.sh 会设为源码 maps/），
        # 未设置时保存到 ~/robot_maps。
        Node(package='fwr_bringup', executable='slam_map_save',
             name='slam_map_save', output='screen'),
        # 后向四路 A22 超声（DYP-E08 / RS485）：发布 4 路 Range
        Node(package='fwr_bringup', executable='ultrasonic_rear',
             name='ultrasonic_rear', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        # 车尾超声守卫：近障时禁止后退（不产生任何前进/倒车指令）
        Node(package='fwr_bringup', executable='rear_guard',
             name='rear_guard', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
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
