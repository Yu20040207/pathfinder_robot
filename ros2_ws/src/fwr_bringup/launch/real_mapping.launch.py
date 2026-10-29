"""Real C1 + encoder + IMU mapping; base motion remains disabled by default."""

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
        DeclareLaunchArgument('start_slam', default_value='true'),
        DeclareLaunchArgument('slam_executable', default_value='async_slam_toolbox_node'),
        DeclareLaunchArgument('slam_params',
                              default_value=str(share / 'config' / 'real_slam.yaml')),
        Node(package='fwr_bringup', executable='base_bridge', name='base_bridge',
             output='screen', parameters=[{'motion_enabled': False,
                                           'use_sim_time': use_sim_time}]),
        Node(package='fwr_imu', executable='dm_imu_node', name='dm_imu_l1',
             condition=IfCondition(LaunchConfiguration('start_imu')), output='screen', parameters=[{'use_sim_time': use_sim_time}]),
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
                          'rear_half_angle_rad': 1.5707963267948966}]),
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
    ])
