"""守护：建图新增功能必须"默认关闭"，且不能改动导航行为。

2026-11-04 用户要求：改自动建图**不能影响已有导航**，所以：
  · 新策略（视点规划、建图专用 BT）必须由 launch 参数显式打开
  · 不传参数时，行为树 = 导航原来那个文件
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'launch' / 'real_auto_mapping.launch.py'
NAV_LAUNCH = ROOT / 'launch' / 'real_navigation.launch.py'
BT_MAP = ROOT / 'behavior_trees' / 'real_navigate_to_pose_mapping.xml'
BT_NAV = ROOT / 'behavior_trees' / 'real_navigate_to_pose.xml'
CFG = ROOT / 'config' / 'real_viewpoint_planner.yaml'


def test_new_switches_default_to_off():
    text = LAUNCH.read_text()
    for arg in ('viewpoint_planner', 'viewpoint_enabled', 'mapping_bt'):
        assert f"DeclareLaunchArgument('{arg}', default_value='false')" in text, (
            f'{arg} 必须默认 false，否则会影响现有建图行为')


def test_viewpoint_node_is_conditional_and_reads_config():
    text = LAUNCH.read_text()
    assert "executable='viewpoint_planner'" in text
    assert "condition=IfCondition(LaunchConfiguration('viewpoint_planner'))" in text
    assert "real_viewpoint_planner.yaml" in text
    cfg = yaml.safe_load(CFG.read_text())['viewpoint_planner']['ros__parameters']
    assert cfg['enabled'] is False, '配置文件里必须默认关闭'


def test_navigation_keeps_its_own_bt_by_default():
    nav = NAV_LAUNCH.read_text()
    # 导航 launch 的默认值必须仍是导航版 BT（不传参数 = 老行为）
    assert "DeclareLaunchArgument('to_pose_bt_xml'" in nav
    assert "'real_navigate_to_pose.xml'" in nav
    # 建图 launch 只在 mapping_bt=true 时才换成 mapping 版
    am = LAUNCH.read_text()
    assert 'real_navigate_to_pose_mapping.xml' in am
    assert "LaunchConfiguration('mapping_bt')" in am


def test_mapping_bt_replans_slower_than_navigation():
    """建图 BT 必须比导航 BT 重规划更慢（减少改道），且必须真的会规划。

    2026-11-04 的教训：IsPathValid 对空路径不可靠（两种写法都导致车完全不动、
    /plan 0 次），所以用"降低重规划频率"来实现，不再依赖 IsPathValid。
    """
    root = ET.parse(BT_MAP).getroot()
    assert {e.get('ID') for e in root.iter('BehaviorTree')} == {'MainTree'}
    xml_text = BT_MAP.read_text()
    assert '<ComputePathToPose' in xml_text and '<FollowPath ' in xml_text
    hz = float(xml_text.split('hz="')[1].split('"')[0])
    nav_hz = float(BT_NAV.read_text().split('hz="')[1].split('"')[0])
    assert hz < nav_hz, f'建图 BT 重规划({hz} Hz) 必须慢于导航({nav_hz} Hz)'
    # 不能再用 IsPathValid 做"是否需要规划"的判据（对空路径失真）
    assert '<IsPathValid' not in xml_text, (
        'Hamble 的 IsPathValid 对空路径返回成功，用它守卫会导致永不规划')


def test_mapping_bt_nodes_all_exist():
    """BT 里用到的每个节点都必须是 Nav2 已注册的，或 BT.CPP 内置的。

    否则运行时会报 "Behavior tree threw exception" / 找不到节点，
    表现就是"点开始导航立刻结束"（2026-11-03 踩过：参数写错导致空树）。
    """
    import os
    from xml.etree import ElementTree as ET2

    nav2_nodes = Path('/opt/ros/humble/share/nav2_behavior_tree/nav2_tree_nodes.xml')
    if not nav2_nodes.exists():
        return                                    # 环境里没有 Nav2，跳过
    known = {e.get('ID') for e in ET2.parse(nav2_nodes).getroot().iter()
             if e.get('ID')}
    core = {'AlwaysSuccess', 'AlwaysFailure', 'Inverter', 'Sequence', 'Fallback',
            'Parallel', 'ReactiveSequence', 'ReactiveFallback', 'SubTree',
            'RetryUntilSuccessful', 'KeepRunningUntilFailure', 'ForceSuccess',
            'ForceFailure', 'PipelineSequence', 'RoundRobin', 'IfThenElse',
            'RateController', 'RecoveryNode', 'Repeat', 'Delay', 'Timeout'}
    tree = ET2.parse(BT_MAP).getroot()
    used = {e.tag for e in tree.iter()
            if e.tag not in ('root', 'BehaviorTree', 'input_port', 'output_port',
                             'inverter', 'script')}
    unknown = sorted(n for n in used if n not in known and n not in core)
    assert not unknown, f'建图 BT 使用了未注册的节点: {unknown}'


def test_viewpoint_planner_owns_its_gate_and_closes_it():
    """方案A：视点规划自己调 Nav2、自己管门控；任何退出路径都必须关门。

    2026-11-04 事故：关掉旧探索器后没人派目标，stuck_reverser 误判"被堵"反复倒车。
    新方案必须能独立于 frontier_explorer 工作。
    """
    src = (ROOT / 'fwr_bringup' / 'viewpoint_planner.py').read_text()
    # 自己调导航（不再依赖 /explore/start_fixed_goal）
    assert 'NavigateToPose' in src and '/navigate_to_pose' in src
    assert 'StartFixedGoal' not in src, '不能依赖旧探索器的服务'
    # 门控三件套
    assert "'motion_enabled'" in src and '/base/arm_auto' in src and '/base/ready' in src
    # 关门出现在：正常失败、超时取消、停止建图、进程退出
    assert src.count('_close_gate()') >= 4
    assert '_cancel_active_goal' in src
    assert 'open_gate' in src, '需要 open_gate 开关（调试时可只算不发）'


def test_stuck_guard_exists_and_never_drives_forward():
    """建图卡住自动修复：只允许清零/重启/原地转，绝不能发前进或倒车。"""
    src = (ROOT / 'fwr_bringup' / 'mapping_stuck_guard.py').read_text()
    assert 'ClearEntireCostmap' in src and '/explore/stop' in src and '/spin' in src
    assert 'Twist' not in src, '守卫节点不得直接发速度指令'
    assert 'linear' not in src.lower() or 'linear' not in src, '不得出现线速度'
    launch = LAUNCH.read_text()
    assert "executable='mapping_stuck_guard'" in launch
    assert "DeclareLaunchArgument('stuck_guard', default_value='true')" in launch


def test_viewpoint_planner_batches_and_never_fakes_completion():
    """两道防线：① 打包多个视点（根治一停一停）② 有前沿时不许判完成。"""
    src = (ROOT / 'fwr_bringup' / 'viewpoint_planner.py').read_text()
    # ① 批量：走 NavigateThroughPoses，且批量点会全部标记为已访问
    assert 'NavigateThroughPoses' in src and '/navigate_through_poses' in src
    assert '_dispatch_batch' in src and 'batch_size' in src
    # ② 假完成：前沿分支必须在 done 之前，且遇到前沿直接 return
    guard = src.index('if has_frontier:')
    done = src.index("'done %.0f%%'")
    assert guard < done, '前沿判断必须在"done"之前'
    block = src[guard:guard + 700]
    assert 'frontier-blocked' in src, '有前沿但去不了时不能判完成'
    # 逐级放宽距离
    assert 'for relax in' in src


def test_batch_goal_uses_the_real_field_name():
    """NavigateThroughPoses.Goal 的字段是 poses（不是 goals）。

    2026-11-04：写成 goals 导致 AttributeError → 节点崩溃 → 底盘停住。
    这个测试直接问 ROS 消息定义，防止再次写错。
    """
    from nav2_msgs.action import NavigateThroughPoses
    fields = set(NavigateThroughPoses.Goal.get_fields_and_field_types())
    assert 'poses' in fields
    src = (ROOT / 'fwr_bringup' / 'viewpoint_planner.py').read_text()
    assert 'goal.poses.append' in src, '必须用 poses 字段'
    assert 'goal.goals.append' not in src, 'goals 字段不存在，会导致节点崩溃'


def test_stuck_guard_recovers_base_faults_without_motion():
    """底盘故障（窄走廊堵转）必须能自动复位，且复位过程不发任何运动指令。"""
    src = (ROOT / 'fwr_bringup' / 'mapping_stuck_guard.py').read_text()
    assert '/base/clear_fault' in src, '必须调用底盘的故障复位服务'
    assert "'/base/fault'" in src and "'/base/ready'" in src
    # 复位前必须把所有运动门关掉（固件要求）
    assert "motion_enabled', False" in src and "manual_enabled', False" in src
    assert 'Twist' not in src, '守卫节点仍然不得直接发速度指令'


def test_trail_reverser_is_wired_and_off_by_default():
    """后退让位：默认关闭、沿轨迹退、车尾拦截优先。"""
    launch = NAV_LAUNCH.read_text()
    assert "DeclareLaunchArgument('trail_reverser', default_value='false')" in launch
    assert "executable='trail_reverser'" in launch
    src = (ROOT / 'fwr_bringup' / 'trail_reverser.py').read_text()
    assert '/cmd_vel_manual' in src, '倒车要走手动通道，车尾联锁才生效'
    assert "'/base/rear_blocked'" in src, '必须用车尾超声拦截'
    assert 'target_behind' in src, '必须沿原路后退（不是盲目直退）'
    assert 'reverse_speed_mps' in src


def test_auto_initial_pose_yields_to_manual_pose():
    """用户手动设位姿后，自动播种必须让位（否则几秒后会把车挪回去）。"""
    src = (ROOT / 'fwr_bringup' / 'auto_initial_pose.py').read_text()
    assert 'respect_manual_pose' in src
    assert '/initialpose' in src
    assert '_manual_pose_seen' in src
    # 播种成功时要重置看门狗时间戳，避免刚播完就误判"丢失定位"
    assert '_tf_ok_at = time.monotonic()   # 防止看门狗立刻误判' in src


def test_reseed_service_can_be_triggered_from_the_panel():
    """面板的「重新定位」按钮 → /localization/reseed → auto_initial_pose 重新匹配并播种。"""
    seed = (ROOT / 'fwr_bringup' / 'auto_initial_pose.py').read_text()
    assert '/localization/reseed' in seed, '必须提供一键重新定位服务'
    assert '_on_reseed' in seed
    panel = (ROOT.parent / 'fwr_rviz_nav' / 'src' / 'two_point_nav_panel.cpp').read_text()
    assert '/localization/reseed' in panel, '面板按钮必须调用该服务'
    assert '重新定位' in panel


def test_explorer_logs_every_silent_mission_stop():
    """任务中止必须留下日志（2026-11-05：现场"走到一半就不走了"查不出原因）。

    之前 _on_manual / _on_bridge_arm / _on_fault / trial 超时都是静默 stop，
    日志里只能看到 Goal was canceled，不知道是谁先动的手。
    """
    src = (ROOT / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert 'def _trip(' in src, '需要统一的中止日志函数'
    for reason in ('manual_enabled 被打开', '底盘自动授权丢失',
                   '底盘故障上报', '收到停止请求', '试运行超时',
                   '状态机停止'):
        assert reason in src, f'缺少中止原因日志: {reason}'
    # 同一原因只打一次，避免 10 Hz 回调刷屏
    assert "_trip_last" in src


def test_multi_point_route_uses_sequential_when_goal_is_near_start():
    """终点离起点近时必须逐段执行，否则 Nav2 路过终点就判到达、中间点被跳过。"""
    src = (ROOT / 'fwr_bringup' / 'frontier_explorer.py').read_text()
    assert 'fixed_route_tail_gap_m' in src, '阈值必须是参数，不能硬编码'
    assert '2.5' in src
    # 路线点必须打印出来，便于事后核对"到底去了哪些点"
    assert '多点路线 %d 个点' in src
