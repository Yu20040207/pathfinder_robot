"""RTAB-Map 建图（雷达 ICP 里程计 + 激光回环闭合）。

与 slam_toolbox / Cartographer 方案的根本区别：
- **不启动 EKF，不用 IMU 绝对航向、不用轮式航向**。里程计由
  `rtabmap_odom/icp_odometry` 直接用雷达扫描做 ICP 配准得到，
  从根上避开我们实测到的 ~2% 航向尺度误差（转 360° 丢 5~7°）。
- 回环闭合用 RTAB-Map 的 BoW + 位姿图优化，全局纠偏。

依赖：sudo apt install -y ros-humble-rtabmap-ros
用法：ros2 launch fwr_bringup real_mapping_rtabmap.launch.py
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fwr_bringup'))
    description = (share / 'description' / 'fwr_chassis_demo.urdf').read_text()
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_port = LaunchConfiguration('lidar_port')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e00aaf475431f111b7818c5f30d20014-if00-port0'),
        Node(package='fwr_bringup', executable='base_bridge', name='base_bridge',
             output='screen',
             parameters=[{'motion_enabled': False, 'use_sim_time': use_sim_time}]),
        # IMU：不作为建图航向（那有 ~2% 尺度误差），但 base_bridge 的传感器就绪门
        # 和手动直行航向保持需要它，缺了会导致键盘指令被静默丢弃。
        Node(package='fwr_imu', executable='dm_imu_node', name='dm_imu_l1',
             output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             output='screen',
             parameters=[{'channel_type': 'serial', 'serial_port': lidar_port,
                          'serial_baudrate': 460800, 'frame_id': 'laser_front',
                          'inverted': False, 'angle_compensate': True,
                          'scan_mode': 'Standard', 'use_sim_time': use_sim_time}]),
        Node(package='fwr_bringup', executable='scan_mapping_filter',
             name='scan_mapping_filter', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'rear_half_angle_rad': 1.5707963267948966}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}]),
        # 雷达 ICP 里程计：只用扫描点云，不碰 IMU/轮式航向
        Node(package='rtabmap_odom', executable='icp_odometry', name='icp_odometry',
             output='screen',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'frame_id': 'base_footprint',
                 'odom_frame_id': 'odom',
                 'publish_tf': True,
                 'wait_for_transform': 0.3,
                 'approx_sync': True,
                 'subscribe_scan': True,
                 'Icp/PointToPlane': 'true',
                 'Icp/Iterations': '30',
                 'Icp/MaxCorrespondenceDistance': '0.3',
                 'Icp/Epsilon': '0.0001',
                 'Icp/PointToPlaneK': '10',
                 'Icp/PointToPlaneMinComplexity': '0.02',
                 # F2M（帧对地图）而不是 F2F（帧对帧）：转弯时点云重叠少，
                 # F2F 会丢配准导致航向误差累积；F2M 用局部地图配准，转弯更稳。
                 'Odom/Strategy': '0',
                 'OdomF2M/MaxSize': '3000',
                 'OdomF2M/ScanMaxSize': '3000',
                 'OdomF2M/ScanSubtractRadius': '0.08',
                 'Odom/ResetCountdown': '1',
                 'Odom/GuessMotion': 'true',
                 'qos_scan': 2,
             }],
             remappings=[('scan', '/scan_mapping')]),
        # RTAB-Map 建图 + 回环闭合
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
             output='screen',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'frame_id': 'base_footprint',
                 'odom_frame_id': 'odom',
                 'map_frame_id': 'map',
                 'subscribe_scan': True,
                 'subscribe_depth': False,
                 'subscribe_rgb': False,
                 'subscribe_stereo': False,
                 'qos_scan': 2,
                 'approx_sync': True,
                 'wait_for_transform': 0.3,
                 'database_path': '/tmp/fwr_rtabmap.db',
                 'delete_db_on_start': True,
                 'Reg/Strategy': '1',
                 'Reg/Force3DoF': 'true',
                 'LaserScan/Normalize': 'true',
                 'LaserScan/MaxRange': '12.0',
                 'Grid/Sensor': '0',
                 'Grid/RangeMax': '12.0',
                 'Grid/CellSize': '0.05',
                 'Grid/ErodeRadius': '0',
                 'RGBD/NeighborLinkRefining': 'true',
                 'RGBD/AngularUpdate': '0.05',
                 'RGBD/LinearUpdate': '0.02',
                 'Optimizer/Strategy': '1',
                 'Mem/IncrementalMemory': 'true',
                 'Rtabmap/DetectionRate': '1.0',
                 'Vis/FeatureType': '0',
             }],
             remappings=[('scan', '/scan_mapping')],
             arguments=['-d']),
    ])
