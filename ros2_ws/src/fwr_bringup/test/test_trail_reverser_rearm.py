"""倒车结束后必须把自动授权还回去。

2026-11-18 发现的缺口：
  trail_reverser 开手动通道 → base_bridge 撤销自动授权（_auto_armed=False）
  _finish() 只把 manual_enabled 设回 False，**没有恢复 _auto_armed**
  → 倒出来之后 Nav2 的指令全被拒 → 车不动，看着像"倒完就卡住"。
"""
from pathlib import Path

SRC = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'trail_reverser.py').read_text()


def test_trail_reverser_watches_auto_armed():
    """必须知道倒车前是否已授权，否则无法判断要不要恢复。"""
    assert "/base/auto_armed" in SRC
    assert '_armed_before' in SRC


def test_finish_requests_rearm():
    """结束时按原状态恢复自动授权。"""
    assert '/base/arm_auto' in SRC
    assert '_restore_arm' in SRC
    assert 'restore_auto_arm' in SRC


def test_rearm_only_when_previously_armed():
    """倒车前没授权（例如人手动开的）就不该擅自授权 —— 安全边界。"""
    i = SRC.index('self._armed_before = self._auto_armed')
    assert i > 0, '必须在开始倒车时记录原状态'
    j = SRC.index('def _finish')
    body = SRC[j:j + 700]
    assert 'self._armed_before' in body, '_finish 要用记录的原始状态做条件'


def test_scan_fail_safe_kept():
    """车尾判定仍然必须走 /base/rear_blocked。"""
    assert "'/base/rear_blocked'" in SRC
