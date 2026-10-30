"""沿墙建图：Cartographer 建图 + 沿墙行进（boundary tracing）。

与 real_auto_mapping（前沿探索）的区别：
  · 不做 frontier exploration（那会在原地左右转、来回折返）
  · 贴着指定一侧的墙走，保持固定距离，边走边扫，走一圈闭环后停

用法：
  ros2 launch fwr_bringup real_wall_mapping.launch.py
  ros2 launch fwr_bringup real_wall_mapping.launch.py side:=left speed:=0.10
停止：Ctrl+C 或 bash tools/start_mapping.sh stop
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
        DeclareLaunchArgument('side', default_value='right'),
        DeclareLaunchArgument('speed', default_value='0.12'),
        DeclareLaunchArgument('target_dist', default_value='0.55'),
        # 建图栈：底盘 + 雷达 + IMU + EKF + Cartographer + 栅格发布
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / 'launch' / 'real_mapping_cartographer.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        # 沿墙控制（自己打开运动门）
        Node(package='fwr_bringup', executable='wall_follower',
             name='wall_follower', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'side': LaunchConfiguration('side'),
                          'speed_mps': LaunchConfiguration('speed'),
                          'target_dist_m': LaunchConfiguration('target_dist')}]),
    ])
