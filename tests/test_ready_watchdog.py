"""导航栈"一直起不来"的自动补齐（2026-11-25）。

现场（11-25 08:37 冷启动）：40+ 节点同时向 DDS 注册，`/amcl/change_state`
应答超时 → `lifecycle_manager` **超时一次就放弃且不重试** →
`map_server`/`amcl` 永远 inactive、Nav2 几个停在 unconfigured，
手机端「导航栈正在启动」永远不消失。

实测正常启动只要 **6 s** 就全 active（11-24 11:10:21 → 11:10:26），
所以这里把宽限压到 **12 s**（首次）/ **8 s**（运行中掉线），
到点就补 configure/activate —— **只改生命周期状态，不杀进程、不发速度**。
"""

from __future__ import annotations

import ast
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
WATCHDOG = ROOT / 'tools/chain_watchdog.py'


def _load_function(name: str):
    tree = ast.parse(WATCHDOG.read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns: dict = {}
            exec(compile(mod, str(WATCHDOG), 'exec'), ns)   # noqa: S102
            return ns[name]
    raise AssertionError(f'{WATCHDOG} 里没有 {name}')


def _module_const(name: str):
    tree = ast.parse(WATCHDOG.read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == name:
                    return ast.literal_eval(node.value)
    raise AssertionError(f'没有常量 {name}')


ready_due = _load_function('ready_due')


# ------------------------------------------------------------ 纯判定
def test_active_needs_nothing():
    assert not ready_due('active', False, 0.0, 100.0, 12, 8, 0.0, 20)


def test_unknown_state_is_left_alone():
    # 状态查询还没回来时不能瞎猜（CLI/服务抖动会导致误判）
    assert not ready_due(None, False, 0.0, 100.0, 12, 8, 0.0, 20)


def test_never_active_waits_12s():
    assert not ready_due('inactive', False, 0.0, 11.9, 12, 8, -60.0, 20)
    assert ready_due('inactive', False, 0.0, 12.0, 12, 8, -60.0, 20)


def test_drop_after_active_waits_only_8s():
    # 运行中掉线要恢复得更快（安全链/平移都靠这些节点）
    assert not ready_due('inactive', True, 0.0, 7.9, 12, 8, -60.0, 20)
    assert ready_due('inactive', True, 0.0, 8.0, 12, 8, -60.0, 20)


def test_cooldown_blocks_repeat_fix():
    # 同一节点刚补过（5 s 前）→ 冷却内不再动，别把生命周期状态机刷坏
    assert not ready_due('inactive', False, 0.0, 100.0, 12, 8, 95.0, 20)
    assert ready_due('inactive', False, 0.0, 100.0, 12, 8, 75.0, 20)


def test_unconfigured_also_counts_as_not_ready():
    assert ready_due('unconfigured', False, 0.0, 13.0, 12, 8, -60.0, 20)


# ------------------------------------------------------------ 常量/接线
def test_ready_nodes_cover_the_stuck_ones():
    nodes = _module_const('READY_NODES')
    for must in ('map_server', 'amcl', 'controller_server',
                 'planner_server', 'behavior_server'):
        assert must in nodes, nodes


def test_safety_monitor_not_in_ready_nodes():
    """READY_NODES 走的是"只补激活"，绝不能把安全链监控混进去被 _fix 杀进程。"""
    assert 'collision_monitor_forward' not in _module_const('READY_NODES')
    assert 'collision_monitor_forward' in _module_const('MONITORS')
    for n in _module_const('READY_NODES'):
        assert n not in _module_const('MONITORS')


def test_watch_ready_wired_into_health():
    src = WATCHDOG.read_text(encoding='utf-8')
    assert '_watch_ready(now)' in src
    assert '_poll_ready(now)' in src


# ------------------------------------------------------------ 错峰启动
def test_launch_staggers_heavy_nodes():
    f = ROOT / 'ros2_ws/src/fwr_bringup/launch/real_navigation.launch.py'
    src = f.read_text(encoding='utf-8')
    assert 'TimerAction' in src
    delayed = re.findall(r'_late\(([\d.]+), Node\(package=\'fwr_bringup\', executable=\'([a-z_]+)\'',
                         src)
    names = {n for _p, n in delayed}
    for n in ('frontier_explorer', 'trail_reverser', 'pose_checkpoint',
              'room_labels', 'map_autosave'):
        assert n in names, f'{n} 没有错峰：{sorted(names)}'
    for p, _n in delayed:
        assert 0 < float(p) <= 8.0, f'错峰太长（会丢指令）：{p}s'


@pytest.mark.parametrize('keep', ['collision_monitor', 'auto_initial_pose',
                                  'auto_activate_nav', 'directional_safety_selector'])
def test_launch_keeps_safety_nodes_immediate(keep):
    """安全链/定位播种必须立刻起，不能错峰。"""
    src = (ROOT / 'ros2_ws/src/fwr_bringup/launch/real_navigation.launch.py'
           ).read_text(encoding='utf-8')
    for m in re.finditer(r'^(\s*)_late\([\d.]+, Node\(package=\'fwr_bringup\', '
                         r'executable=\'([a-z_]+)\'', src, re.M):
        assert keep not in m.group(2), f'{keep} 不该错峰'
