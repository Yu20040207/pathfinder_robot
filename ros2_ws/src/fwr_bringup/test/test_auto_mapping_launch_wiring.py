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


def test_mapping_bt_uses_conditional_replanning():
    root = ET.parse(BT_MAP).getroot()
    ids = {e.get('ID') for e in root.iter('BehaviorTree')}
    assert 'MainTree' in ids
    xml_text = BT_MAP.read_text()
    # 核心：IsPathValid 守护重规划 + 仍有 FollowPath
    assert '<IsPathValid' in xml_text, '建图 BT 必须用 IsPathValid 做条件重规划'
    assert 'ComputePathToPose' in xml_text
    assert 'FollowPath' in xml_text
    assert 'AlwaysSuccess' in xml_text, '路径有效时要能直接通过（不改道）'
    # 导航 BT 必须保持原样（不含 IsPathValid 元素；注释里提到过不算）
    nav_bt = BT_NAV.read_text()
    assert '<IsPathValid' not in nav_bt, '导航 BT 不能被改动'
    assert '<ComputePathToPose' in nav_bt and '<FollowPath ' in nav_bt
