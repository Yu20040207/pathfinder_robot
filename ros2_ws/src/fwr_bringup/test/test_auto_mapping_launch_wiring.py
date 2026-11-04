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
