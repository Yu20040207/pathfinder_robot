"""底盘故障锁存自愈（2026-11-28 现场："手动模式又显示等待传感器"）。

现场事实：`/base/motor_debug` 末位（固件 fault）实测 **0.0**，固件早就清干净了，
但 ROS 侧 `core.fault` 一直停在 1 → `/base/ready=false` → 页面「已授权（等待传感器）」，
日志每 2 s 刷一次 `auto-clearing latched ESP32 fault 1`，只能重启节点。

根因：两处"自动清故障"只往串口写了 `b'xc'`，**没有置 `_fault_clear_pending`**；
而 `_on_state` 只在 `_fault_clear_pending` 为真时才接受 fault=0（否则保留锁存值，
防止把真故障当成已恢复）。于是固件清了、本地永远不清 → 死循环。
"""

from __future__ import annotations

import pathlib

from fwr_bringup.base_bridge_core import BaseBridgeCore
from fwr_bringup.base_protocol import BaseState


def _state(fault: int, esp_us: int) -> BaseState:
    return BaseState(esp_us, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     False, fault)


def _core() -> BaseBridgeCore:
    core = BaseBridgeCore()
    core.connect(0.0)
    core.verified = True
    return core


def test_fault_latches_until_clear_is_recorded() -> None:
    """固件报 1 → 本地锁存；没登记过清故障，固件报 0 也不许自动放行。"""
    core = _core()
    core.step(_state(1, 1_000_000), 0.0)
    assert core.fault == 1
    assert core.ready(0.0) is False

    # 没有登记"已发过 xc"：固件报 0 时保留锁存（这是刻意的安全语义）
    core.step(_state(0, 1_100_000), 0.1)
    assert core.fault == 1


def test_note_fault_clear_sent_lets_next_clean_frame_clear_latch() -> None:
    """登记过 xc 之后，下一帧 fault=0 必须落到本地 → ready 恢复。"""
    core = _core()
    core.step(_state(1, 1_000_000), 0.0)
    assert core.fault == 1
    assert core.fault_clear_pending is False

    core.note_fault_clear_sent()
    assert core.fault_clear_pending is True

    core.step(_state(0, 1_100_000), 0.1)
    assert core.fault == 0
    assert core.fault_clear_pending is False
    assert core.ready(0.1) is True


def test_note_fault_clear_sent_does_not_hide_a_new_fault() -> None:
    """登记后固件仍报故障（清失败/新故障）→ 保持故障，不许乐观清零。"""
    core = _core()
    core.step(_state(1, 1_000_000), 0.0)
    core.note_fault_clear_sent()
    core.step(_state(3, 1_100_000), 0.1)
    assert core.fault == 3
    assert core.ready(0.1) is False


def test_node_records_every_self_sent_fault_clear() -> None:
    """静态钉住：节点里两处自动清故障都必须登记，且节流条件要看 pending。"""
    src = pathlib.Path('ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge.py').read_text(
        encoding='utf-8')
    # 三处写 b'xc'：/base/clear_fault 服务（走 request_fault_clear，自带 pending）
    # + 连上自动清 + 运行中兜底。后两处必须显式登记。
    assert src.count("b'xc'") == 3, '写 xc 的位置应为：服务 1 处 + 自动清 2 处'
    assert 'self._core.request_fault_clear(now)' in src, '服务通道应走 request_fault_clear'
    assert src.count('self._core.note_fault_clear_sent()') == 2, (
        '两处自动清 xc 之后都必须登记，否则本地锁存永远清不掉（2026-11-28）')
    assert 'not self._core.fault_clear_pending' in src, (
        '运行中兜底要看 pending，避免每 2 s 重复写 xc、刷日志')
