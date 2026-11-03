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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fwr_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # slam_engine: slam（slam_toolbox，**默认**）或 carto
        # 2026-11-01：实车对比后改用 slam_toolbox —— 用户反馈 Cartographer 走过的地方
        # 形不成闭环（墙对不上），slam_toolbox 用 scan matching + 位姿图，闭环正常。
        # carto 仍可用 `slam_engine:=carto` 切回。
        DeclareLaunchArgument('slam_engine', default_value='slam'),
        # 2026-10-31：自动建图必须同时把 RViz 打开（用户要求现场能看到地图和规划）
        DeclareLaunchArgument('start_rviz', default_value='true'),
        # 2026-10-31：补扫模式用 —— 关掉自动探索，手动发目标
        DeclareLaunchArgument('use_explorer', default_value='true'),
        DeclareLaunchArgument('initial_spin', default_value='true'),
        # 探索器延迟启动，留出自转时间（自转约 21 s + 启动余量）
        DeclareLaunchArgument('explorer_start_delay_s', default_value='32.0'),
        # 2026-10-31：探索策略
        #   room     = 房间级探索（先把当前房间扫完再出门，默认）
        #   frontier = 全局前沿探索（frontier_exploration_ros2）
        DeclareLaunchArgument('explorer', default_value='room'),
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
        # 房间级探索：先把当前房间扫完整再出门
        # 2026-11-03（用户要求）：建图一开始先原地慢速转一圈，
        # 让 270° 雷达把四周墙面扫进来，再交给探索器。
        # 只转不走、前方过近立即停、用 yaw 计角度。可用 initial_spin:=false 关闭。
        Node(package='fwr_bringup', executable='initial_spin', name='initial_spin',
             output='screen',
             parameters=[{'enabled': LaunchConfiguration('initial_spin')}]),
        # 2026-11-03：room_explorer 会**自己直接驱动底盘**（往 /cmd_vel_manual
        # 发速度、自己开关运动门），一启动就和 initial_spin 抢同一条通道，
        # 表现为"刚启动就往前走、不转圈"。这里延迟启动，等自转结束后再接管。
        TimerAction(
            period=LaunchConfiguration('explorer_start_delay_s'),
            actions=[Node(
                package='fwr_bringup', executable='room_explorer',
                name='room_explorer', output='screen',
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('use_explorer'), "' == 'true' and '",
                    LaunchConfiguration('explorer'), "' == 'room'"])),
                parameters=[{'use_sim_time': use_sim_time}]),
            ]),
        Node(package='frontier_exploration_ros2', executable='frontier_explorer',
             name='frontier_explorer', output='screen',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('use_explorer'), "' == 'true' and '",
                 LaunchConfiguration('explorer'), "' == 'frontier'"])),
             parameters=[str(share / 'config' / 'real_frontier_exploration.yaml'),
                         {'use_sim_time': use_sim_time}]),
        # 「停止建图」服务：停车 + 结束轨迹 + 保存 pbstream/pgm
        Node(package='fwr_bringup', executable='mapping_save', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        # RViz：显示地图 / 激光 / 规划路径 / 沿墙规划 / 录制路线
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             condition=IfCondition(LaunchConfiguration('start_rviz')),
             arguments=['-d', str(share / 'config' / 'real_localization.rviz')]),
        # 卡死自救：前停区持续拦截且车尾空旷时，主动倒 0.4 m 再交还导航。
        # （脱困原来要等两次目标失败，现场表现为长时间卡死）
        Node(package='fwr_bringup', executable='stuck_reverser',
             name='stuck_reverser', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
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
