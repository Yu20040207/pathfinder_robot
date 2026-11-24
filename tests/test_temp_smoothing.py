"""CPU 温度显示「跳 10 度」的修复（2026-11-24 用户报）。

实测 `x86_pkg_temp` 会偶发单次虚高：同一秒连读 20 次得到
50/51/50/50/68/70/50 °C，而当时整机 CPU 占用稳定 25%、功率没跳 ——
那 18 °C 是采样毛刺，不是真温升。桥接改成「最近 5 个样本取中位数」，
单点毛刺进不来，真实趋势 2~3 s 跟得上。
"""
from fwr_bringup.mqtt_bridge import _TEMP_HIST, smooth_temp


def _reset():
    _TEMP_HIST.clear()


def test_single_spike_is_rejected():
    _reset()
    for v in (50.0, 51.0, 50.0, 51.0, 50.0):
        smooth_temp('cpu', v)
    # 采样毛刺
    assert smooth_temp('cpu', 70.0) <= 52.0
    assert smooth_temp('cpu', 51.0) == 51.0


def test_sustained_rise_is_followed():
    _reset()
    for _ in range(5):
        smooth_temp('cpu', 50.0)
    # 真升温：连续 5 个样本都在 70 附近 → 中位数必须跟上去
    for _ in range(5):
        v = smooth_temp('cpu', 70.0)
    assert v == 70.0


def test_none_passes_through_and_history_is_per_key():
    _reset()
    assert smooth_temp('ssd', None) is None
    for v in (40.0, 41.0, 42.0):
        smooth_temp('cpu', v)
        smooth_temp('ssd', 60.0)
    assert abs(smooth_temp('cpu', 42.0) - 42.0) <= 0.5   # 偶数个样本取中间两数均值
    assert smooth_temp('ssd', 60.0) == 60.0
    _reset()
