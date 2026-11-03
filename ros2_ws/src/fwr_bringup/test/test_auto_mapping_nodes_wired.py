"""回归测试：长期运行的节点必须真的把它的主循环挂到定时器上。

2026-11-03 踩过两次：一次是 initial_spin 的 create_timer 被挤到 return 之后，
一次是 auto_mapping_supervisor 的 create_timer 被正则编辑误删 —— 两次表现都是
"节点启动了、但什么都不做、日志一片空白"，非常难查。
"""
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1] / 'fwr_bringup'

# 这些节点都靠 _tick 周期性干活，必须各自创建定时器
TICK_NODES = ('auto_mapping_supervisor', 'room_explorer')


@pytest.mark.parametrize('name', TICK_NODES)
def test_node_wires_its_tick_to_a_timer(name):
    src = (PKG / f'{name}.py').read_text(encoding='utf-8')
    assert 'def _tick' in src, f'{name} 没有 _tick'
    assert 'create_timer' in src, (
        f'{name} 的 _tick 没有挂到定时器上 —— 节点会启动但永远不干活')
    # 定时器必须在 __init__ 里创建（不能被挤到别的函数/return 后面）
    init = src.split('def _tick')[0]
    assert 'create_timer' in init, f'{name} 的 create_timer 不在 __init__ 里'
