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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fwr_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # slam_engine: carto（默认，推荐） 或 slam
        DeclareLaunchArgument('slam_engine', default_value='carto'),
        # 2026-10-31：自动建图必须同时把 RViz 打开（用户要求现场能看到地图和规划）
        DeclareLaunchArgument('start_rviz', default_value='true'),
        # 2026-10-31：补扫模式用 —— 关掉自动探索，手动发目标
        DeclareLaunchArgument('use_explorer', default_value='true'),
        DeclareLaunchArgument('load_state', default_value=''),
        # 建图 + Nav2 + 门控/碰撞监控/方向选择器（复用 real_navigation 的 use_saved_map:=false 分支）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'real_navigation.launch.py')),
            launch_arguments={'use_saved_map': 'false',
                              'use_sim_time': use_sim_time,
                              'start_explorer': 'false',
                              # 2026-10-30：离线 bag 对比后确定用 Cartographer
                              'slam_engine': LaunchConfiguration('slam_engine'),
                              'load_state': LaunchConfiguration('load_state')}.items()),
        # 把机器人脚下涂成空闲再给探索器（见节点 docstring）
        Node(package='fwr_bringup', executable='map_free_around_robot',
             name='map_free_around_robot', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        # 前沿探索（2026-10-31 换成 frontier_exploration_ros2）：
        # 比 explore_lite 强的地方：MRTSP 目标排序（不在原地折返）、
        # 支持非 360° 雷达、前沿抑制（门进不去就不反复撞）、自带完成事件。
        Node(package='frontier_exploration_ros2', executable='frontier_explorer',
             name='frontier_explorer', output='screen',
             condition=IfCondition(LaunchConfiguration('use_explorer')),
             parameters=[str(share / 'config' / 'real_frontier_exploration.yaml'),
                         {'use_sim_time': use_sim_time}]),
        # 「停止建图」服务：停车 + 结束轨迹 + 保存 pbstream/pgm
        Node(package='fwr_bringup', executable='mapping_save', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        # RViz：显示地图 / 激光 / 规划路径 / 沿墙规划 / 录制路线
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             condition=IfCondition(LaunchConfiguration('start_rviz')),
             arguments=['-d', str(share / 'config' / 'real_localization.rviz')]),
        # 链路看门狗：碰撞监控卡死时自动重启（老毛病，见 AGENTS.md）
        ExecuteProcess(
            cmd=['python3', str(Path(__file__).resolve().parents[4] / 'tools' / 'chain_watchdog.py')],
            name='chain_watchdog', output='screen'),
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
