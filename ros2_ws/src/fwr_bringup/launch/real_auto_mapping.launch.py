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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        # 2026-10-31：探索策略
        #   room     = 房间级探索（先把当前房间扫完再出门，默认）
        #   frontier = 全局前沿探索（frontier_exploration_ros2）
        DeclareLaunchArgument('explorer', default_value='room'),
        DeclareLaunchArgument('load_state', default_value=''),
        # 2026-11-04 视点规划（新策略）：
        #   viewpoint_planner=true   → 启动视点规划节点
        #   viewpoint_enabled=true   → 它才真正开始派目标（默认 false = 只看不动）
        # 两个都默认 false ⇒ 不传参数时行为与之前**完全一致**。
        DeclareLaunchArgument('viewpoint_planner', default_value='false'),
        DeclareLaunchArgument('viewpoint_enabled', default_value='false'),
        # 建图专用行为树（条件重规划，防中途改道）。默认 = 导航版，行为不变。
        DeclareLaunchArgument('mapping_bt', default_value='false'),
        # stuck_reverser 会把"没人派目标"误判成"被堵"然后倒车。用视点规划时可以关掉。
        DeclareLaunchArgument('stuck_reverser', default_value='true'),
        # kickstart（直行+倒车退回）开关；用视点规划时可以关掉，只靠导航转弯
        DeclareLaunchArgument('kickstart', default_value='true'),
        DeclareLaunchArgument('kick_reverse', default_value='true'),
        # 建图卡住自动修复（默认开：用户要求这类卡死不再需要人工介入）
        DeclareLaunchArgument('stuck_guard', default_value='true'),
        # 建图 + Nav2 + 门控/碰撞监控/方向选择器（复用 real_navigation 的 use_saved_map:=false 分支）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'real_navigation.launch.py')),
            launch_arguments={'use_saved_map': 'false',
                              'use_sim_time': use_sim_time,
                              'start_explorer': 'false',
                              # 2026-10-30：离线 bag 对比后确定用 Cartographer
                              'slam_engine': LaunchConfiguration('slam_engine'),
                              'load_state': LaunchConfiguration('load_state'),
                              # mapping_bt:=true 才换成"条件重规划"的建图专用 BT
                              'to_pose_bt_xml': PythonExpression([
                                  "'", str(share / 'behavior_trees' / 'real_navigate_to_pose_mapping.xml'),
                                  "' if '", LaunchConfiguration('mapping_bt'), "' == 'true' else '",
                                  str(share / 'behavior_trees' / 'real_navigate_to_pose.xml'), "'"]),
                              }.items()),
        # 把机器人脚下涂成空闲再给探索器（见节点 docstring）
        Node(package='fwr_bringup', executable='map_free_around_robot',
             name='map_free_around_robot', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        # 前沿探索（2026-10-31 换成 frontier_exploration_ros2）：
        # 比 explore_lite 强的地方：MRTSP 目标排序（不在原地折返）、
        # 支持非 360° 雷达、前沿抑制（门进不去就不反复撞）、自带完成事件。
        # 房间级探索：先把当前房间扫完整再出门
        # 2026-11-03（用户思路）：不用固定延迟 —— room_explorer 订阅
        # /mapping/spin_done，收到 supervisor 的"转完"信号后才接管底盘。
        # 事件驱动，转得快就早开始，转得慢也不会被抢走。
        Node(package='fwr_bringup', executable='room_explorer',
             name='room_explorer', output='both',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('use_explorer'), "' == 'true' and '",
                 LaunchConfiguration('explorer'), "' == 'room'"])),
             parameters=[{
                 'use_sim_time': use_sim_time,
                 # 2026-11-03：关掉探索器自己的"短退脱困"。
                 # 它要 /explore/state='reversing' 才允许后退，但监督器每 50 ms
                 # 就把 state 覆盖成 navigating → 脱困永远退不出去，
                 # 期间 manual_enabled 卡 True → 自动链路被门控永久封锁。
                 'escape_after_failures': 9999,
                 # 决策周期从 3 s 缩到 1.5 s，减少目标之间的空档。
                 'replan_s': 1.5,
                 # 2026-11-03（现场：出了房间到走廊后只在走廊里转，进不去别的房间）：
                 # 房间门控会把走廊当成一个"房间"，只要它还有前沿就永远不去别处。
                 # 关掉 → 用全局前沿，能主动走进其他房间。
                 'use_room_gate': False}]),
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
             # 2026-11-03 修：建图必须用 real_mapping.rviz。
             # real_localization.rviz 里的 Map Edit 面板会持续重发一张旧 /map，
             # 和 slam 的新地图交替刷屏 → 地图尺寸来回跳、看着像"不长"。
             arguments=['-d', str(share / 'config' / 'real_mapping.rviz')]),
        # 卡死自救：前停区持续拦截且车尾空旷时，主动倒 0.4 m 再交还导航。
        # （脱困原来要等两次目标失败，现场表现为长时间卡死）
        Node(package='fwr_bringup', executable='stuck_reverser',
             name='stuck_reverser', output='screen',
             condition=IfCondition(LaunchConfiguration('stuck_reverser')),
             parameters=[{'use_sim_time': use_sim_time}]),
        # 链路看门狗：碰撞监控卡死时自动重启（老毛病，见 AGENTS.md）
        ExecuteProcess(
            cmd=['python3', str(Path(__file__).resolve().parents[4] / 'tools' / 'chain_watchdog.py')],
            name='chain_watchdog', output='screen'),
        # 监督节点：踢一脚小方块让地图长轮廓 → 打开自动授权交给 explore_lite
        # （同时 10 Hz 发布 /explore/state=navigating 保持安全链）
        Node(package='fwr_bringup', executable='auto_mapping_supervisor',
             name='auto_mapping_supervisor', output='both',
             parameters=[{'use_sim_time': use_sim_time,
                          'kick_leg_m': 1.0,
                          'kick_speed_mps': 0.15,
                          'kick_turn_rad_s': 0.7,
                          'startup_delay_s': 15.0,
                          'kick_enabled': PythonExpression([
                              "'", LaunchConfiguration('kickstart'), "' == 'true'"]),
                          'kick_reverse_enabled': PythonExpression([
                              "'", LaunchConfiguration('kick_reverse'), "' == 'true'"])}]),
        # 建图卡住自动修复：位移/地图都不增长时逐级自愈（清代价地图→重启探索→原地转）
        Node(package='fwr_bringup', executable='mapping_stuck_guard',
             name='mapping_stuck_guard', output='both',
             condition=IfCondition(LaunchConfiguration('stuck_guard')),
             parameters=[{'use_sim_time': use_sim_time}]),
        # 视点规划器（默认不启动；视点规划是新策略，旧的 room_explorer 仍可用）
        Node(package='fwr_bringup', executable='viewpoint_planner',
             name='viewpoint_planner', output='both',
             condition=IfCondition(LaunchConfiguration('viewpoint_planner')),
             parameters=[str(share / 'config' / 'real_viewpoint_planner.yaml'),
                         {'use_sim_time': use_sim_time,
                          'enabled': PythonExpression([
                              "'", LaunchConfiguration('viewpoint_enabled'), "' == 'true'"])}]),
    ])
