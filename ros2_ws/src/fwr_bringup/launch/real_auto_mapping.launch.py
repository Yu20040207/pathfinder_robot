"""自动建图：slam 建图 + Nav2 + explore_lite 前沿探索 + 完整安全链。

用法：ros2 launch fwr_bringup real_auto_mapping.launch.py
启动后执行：
  ros2 param set /base_bridge motion_enabled true
  ros2 service call /base/arm_auto std_srvs/srv/Trigger
机器人会自己沿地图未知边界行驶直到没有前沿；结束后 serialize_map + save_map。
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
        # 建图 + Nav2 + 门控/碰撞监控/方向选择器（复用 real_navigation 的 use_saved_map:=false 分支）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'real_navigation.launch.py')),
            launch_arguments={'use_saved_map': 'false',
                              'use_sim_time': use_sim_time,
                              'start_explorer': 'false'}.items()),
        # 前沿探索：向 /navigate_to_pose 发送目标
        Node(package='explore_lite', executable='explore',
             name='explore_node', output='screen',
             parameters=[str(share / 'config' / 'real_auto_mapping.yaml'),
                         {'use_sim_time': use_sim_time}]),
        # 监督节点：踢一脚小方块让地图长轮廓 → 打开自动授权交给 explore_lite
        # （同时 10 Hz 发布 /explore/state=navigating 保持安全链）
        Node(package='fwr_bringup', executable='auto_mapping_supervisor',
             name='auto_mapping_supervisor', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'kick_leg_m': 1.0,
                          'kick_speed_mps': 0.15,
                          'kick_turn_rad_s': 0.7,
                          'startup_delay_s': 15.0}]),
    ])
