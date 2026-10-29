"""用 Google Cartographer 建图（替代 slam_toolbox）。

Cartographer 用子图 + 位姿图 + 在线相关匹配，室内走廊的航向鲁棒性明显好于
slam_toolbox（后者在平行墙走廊里会缓慢旋转）。

依赖：sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
用法：ros2 launch fwr_bringup real_mapping_cartographer.launch.py
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fwr_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # 底盘/雷达/IMU/EKF 照旧，但不启动 slam_toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'real_mapping.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'start_slam': 'false'}.items()),
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
             output='screen',
             arguments=['-configuration_directory', str(share / 'config'),
                        '-configuration_basename', 'fwr_2d.lua'],
             parameters=[{'use_sim_time': use_sim_time}],
             remappings=[('scan', '/scan_mapping'),
                         ('odom', '/odom'),
                         ('imu', '/imu/data_raw')]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='cartographer_occupancy_grid_node', output='screen',
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
