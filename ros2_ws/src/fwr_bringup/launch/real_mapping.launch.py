"""Real C1 + encoder + IMU mapping; base motion remains disabled by default."""

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
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        Node(package='fwr_bringup', executable='base_bridge', name='base_bridge',
             output='screen', parameters=[{'motion_enabled': False,
                                           'use_sim_time': use_sim_time}]),
        Node(package='fwr_imu', executable='dm_imu_node', name='dm_imu_l1',
             output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             output='screen', parameters=[{
                 'channel_type': 'serial', 'serial_port': LaunchConfiguration('lidar_port'),
                 'serial_baudrate': 460800, 'frame_id': 'laser_front',
                 'inverted': False, 'angle_compensate': True,
                 'scan_mode': 'Standard', 'use_sim_time': use_sim_time,
             }]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             parameters=[str(share / 'config' / 'real_ekf.yaml')]),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=[str(share / 'config' / 'real_slam.yaml')]),
    ])
