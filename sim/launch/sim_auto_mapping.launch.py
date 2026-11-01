"""仿真自动建图：四房间世界 + 我们的完整自动建图链。

组成：
  1. sim_bringup（Gazebo 世界/机器人/传感器桥接/静态TF/260°遮罩/超声/EKF/Cartographer）
  2. Nav2（与实机同一份 real_nav2.yaml）
  3. 安全链：auto_command_gate → 2×collision_monitor → directional_safety_selector
  4. 车尾守卫 rear_guard + sim_cmd_router（含车尾联锁）
  5. 房间级探索 room_explorer + auto_mapping_supervisor

用法：
    export DISPLAY=:1
    ros2 launch sim/launch/sim_auto_mapping.launch.py
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
SIM = os.path.join(REPO, 'sim')


def generate_launch_description() -> LaunchDescription:
    fwr = get_package_share_directory('fwr_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    explorer = LaunchConfiguration('explorer')

    params = RewrittenYaml(
        source_file=os.path.join(fwr, 'config', 'real_nav2.yaml'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml':
                os.path.join(fwr, 'behavior_trees', 'real_navigate_to_pose.xml'),
            'default_nav_through_poses_bt_xml':
                os.path.join(fwr, 'behavior_trees', 'real_navigate_through_poses.xml'),
        },
        convert_types=True,
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(SIM, 'launch', 'sim_bringup.launch.py')),
        launch_arguments={'start_rviz': start_rviz,
                          'use_sim_time': use_sim_time}.items())

    # Nav2（不自动激活：交给 auto_activate_nav 等地图就绪）
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': params,
                          'autostart': 'false',
                          'use_composition': 'False'}.items())

    # 地图密封（膨胀 1 格），给代价地图静态层用 —— 与实机同一节点
    # 固定 24x24 m 画布：避免 Cartographer 地图变大 → 全局代价地图反复 resize
    # → SmacPlanner segfault（planner_server exit -11，2026-11-01 实测）
    map_seal = Node(package='fwr_bringup', executable='map_seal',
                    name='map_seal', output='screen',
                    parameters=[{'use_sim_time': use_sim_time,
                                 # 与 real_nav2.yaml 的全局代价地图窗口严格对齐
                                 'fixed_width_m': 44.0,
                                 'fixed_height_m': 40.0,
                                 'fixed_center_x': -2.0,
                                 'fixed_center_y': -12.0,
                                 # 仿真的 Cartographer 地图很干净（没有实机那种 1~3 格缝），
                                 # 不需要膨胀；膨胀会把"离墙 5 cm 的机器人脚下"也标成障碍，
                                 # 规划器随即报 Starting point in lethal space。
                                 'dilate_cells': 0}])

    activate = Node(package='fwr_bringup', executable='auto_activate_nav',
                    name='auto_activate_nav', output='screen',
                    parameters=[{'use_sim_time': use_sim_time}])

    # ---- 安全链（与实机完全相同）
    gate = Node(package='fwr_bringup', executable='auto_command_gate',
                name='auto_command_gate', output='screen',
                parameters=[{'use_sim_time': use_sim_time}])
    # 仿真里没有 base_bridge → 没有人发布 /base/manual_status，
    # 选择器默认 require_manual_status=true 会一直判 stale-manual 输出 0。
    selector = Node(package='fwr_bringup', executable='directional_safety_selector',
                    name='directional_safety_selector', output='screen',
                    parameters=[{'use_sim_time': use_sim_time, 'scan_topic': '/scan',
                                 'require_manual_status': False}])
    cm_fwd = Node(package='nav2_collision_monitor', executable='collision_monitor',
                  name='collision_monitor_forward', output='screen',
                  parameters=[os.path.join(fwr, 'config', 'real_collision_monitor_forward.yaml'),
                              {'use_sim_time': use_sim_time}])
    cm_mgr = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                  name='lifecycle_manager_collision', output='screen',
                  parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                               'node_names': ['collision_monitor_forward']}])

    # ---- 车尾守卫 + 底盘入口
    rear_guard = Node(package='fwr_bringup', executable='rear_guard',
                      name='rear_guard', output='screen',
                      parameters=[{'use_sim_time': use_sim_time}])
    cmd_router = Node(package='fwr_bringup', executable='sim_cmd_router',
                      name='sim_cmd_router', output='screen',
                      parameters=[{'use_sim_time': use_sim_time}])

    # ---- 探索
    # 仿真不需要授权流程（没有 base_bridge / ESP32），
    # 用一个固定发布 /explore/state=navigating 的节点让安全链保持开放。
    # 实机上这一步由 auto_mapping_supervisor 完成。
    state_pub = Node(package='fwr_bringup', executable='auto_explore_state',
                     name='auto_explore_state', output='screen',
                     parameters=[{'use_sim_time': use_sim_time,
                                  'state': 'navigating'}])
    room_explorer = Node(package='fwr_bringup', executable='room_explorer',
                         name='room_explorer', output='screen',
                         condition=IfCondition(explorer),
                         parameters=[{'use_sim_time': use_sim_time}])
    # 链路看门狗：碰撞监控触发一次 stop 后会卡死不再处理指令（老毛病），
    # 卡死时 /cmd_vel_auto_*_filtered 静默 → 车不动。实机一直有，仿真之前漏了。
    watchdog = ExecuteProcess(
        cmd=['python3', os.path.join(REPO, 'tools', 'chain_watchdog.py')],
        name='chain_watchdog', output='screen')

    free_around = Node(package='fwr_bringup', executable='map_free_around_robot',
                       name='map_free_around_robot', output='screen',
                       parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('explorer', default_value='true'),
        sim,
        nav2,
        map_seal, activate,
        gate, selector, cm_fwd, cm_mgr,
        rear_guard, cmd_router,
        watchdog,
        free_around, room_explorer, state_pub,
    ])
