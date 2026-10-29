"""Show the physical C1 scan in the robot's chassis frame."""

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
    serial_port = LaunchConfiguration('serial_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
             output='screen', parameters=[{
                 'channel_type': 'serial', 'serial_port': serial_port,
                 'serial_baudrate': 460800, 'frame_id': 'laser_front',
                 'inverted': False, 'angle_compensate': True,
                 'scan_mode': 'Standard', 'use_sim_time': use_sim_time,
             }]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', parameters=[{
                 'robot_description': description, 'use_sim_time': use_sim_time,
             }]),
        Node(package='rviz2', executable='rviz2', name='c1_real_rviz',
             arguments=['-d', str(share / 'config' / 'lidar_c1_real.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
