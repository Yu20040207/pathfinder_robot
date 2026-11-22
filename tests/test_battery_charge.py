"""充电判定的单元测试（2026-11-22 用户报"充电中一直跳动"）。"""

import random

import pytest

from fwr_bringup.battery_charge import ChargeDetector, slope_v_per_s


def _run(det, series, t0=1000.0, dt=1.0):
    """series: [(voltage, current), ...]，1 Hz；返回每个采样之后的状态。"""
    return [det.feed(t0 + k * dt, v, i) for k, (v, i) in enumerate(series)]


# ------------------------------------------------------------------ 斜率
def test_slope_of_linear_ramp():
    hist = [(0.0 + k, 21.0 + 0.001 * k) for k in range(300)]
    assert slope_v_per_s(hist) == pytest.approx(0.001, abs=2e-5)


def test_slope_none_when_data_too_short():
    assert slope_v_per_s([]) is None
    assert slope_v_per_s([(0.0, 21.0), (1.0, 21.1)]) is None
    assert slope_v_per_s([(0.0 + k, 21.0 + 0.001 * k) for k in range(20)]) is None
    # min_span_s 抬高后，240 s 以下的数据一律"不可信"
    hist = [(0.0 + k, 21.0 + 0.001 * k) for k in range(120)]
    assert slope_v_per_s(hist, min_span_s=240.0) is None


def test_slope_of_flat_noise_is_near_zero():
    rng = random.Random(7)
    hist = [(0.0 + k, 21.4 + rng.gauss(0.0, 0.02)) for k in range(600)]
    assert abs(slope_v_per_s(hist)) < 3e-5


# ------------------------------------------------- 2026-11-22 的跳动必须消失
def test_status_does_not_flap_like_before():
    """旧规则在 21.4 V 时每秒在充电/未充电之间翻；新判定必须**不来回翻**。

    用实测序列特征：电压 21.37~21.42 抖动、电流 1.0~1.5 A，
    真实趋势约 +0.0002 V/s（真在充电）。
    """
    rng = random.Random(11)
    det = ChargeDetector(initial=False)
    seen = []
    for k in range(900):
        v = 21.37 + 0.0002 * k + rng.gauss(0.0, 0.02)
        i = 1.25 + rng.gauss(0.0, 0.15)
        seen.append(det.feed(1000.0 + k, v, i))
    flips = sum(1 for a, b in zip(seen, seen[1:]) if a != b)
    assert flips <= 1, f'状态翻了 {flips} 次'
    assert seen[-1] is True


def test_no_flap_when_flat_at_low_soc():
    """停在 21.4 V 附近不动（既不在充也没在放）→ 保持原状态，一次都不翻。"""
    rng = random.Random(5)
    det = ChargeDetector(initial=False)
    seen = _run(det, [(21.40 + rng.gauss(0.0, 0.02), 1.25) for _ in range(900)])
    assert not any(seen)


def test_charging_starts_only_after_window_and_debounce():
    det = ChargeDetector(debounce_s=30.0)
    seq = [(21.0 + 0.001 * k, 1.2) for k in range(400)]
    out = _run(det, seq)
    assert out[200] is False          # 窗口还不够 240 s → 不猜
    assert out[260] is False          # 刚够窗口，还在去抖
    assert out[-1] is True            # 240 s 窗口 + 30 s 去抖之后才亮


def test_unplug_is_detected_as_discharging():
    det = ChargeDetector(initial=True, debounce_s=30.0)
    seq = [(22.0 - 0.0002 * k, 1.25) for k in range(600)]
    out = _run(det, seq)
    assert out[-1] is False


def test_flat_noisy_voltage_holds_previous_state():
    """斜率落在死区（既不算升也不算降）→ 保持原状态，不乱猜。"""
    rng = random.Random(3)
    det = ChargeDetector(initial=True)
    out = _run(det, [(21.40 + rng.gauss(0, 0.02), 1.2) for _ in range(900)])
    assert all(out)


def test_cv_top_counts_as_charging_even_with_flat_voltage():
    """恒压阶段电压不动，只能靠"到顶 + 有电流"判定。"""
    det = ChargeDetector(top_v=24.60, debounce_s=5.0)
    out = _run(det, [(24.70, 0.8) for _ in range(320)])
    assert out[-1] is True


def test_rise_below_threshold_is_not_charging():
    """每秒 ±0.02 V 的抖动不算"在充"（平滑后斜率 < 8e-5，也没到 CV 顶）。"""
    det = ChargeDetector(initial=False)
    seq = [(21.40 + (0.02 if k % 2 else -0.02), 1.2) for k in range(600)]
    out = _run(det, seq)
    assert not any(out)


def test_window_drops_old_samples():
    det = ChargeDetector(debounce_s=0.0)
    _run(det, [(20.0, 1.0)] * 5, t0=0.0)
    _run(det, [(22.0, 1.0)] * 5, t0=700.0)
    assert all(t >= 700.0 - 600.0 for t, _ in det._hist)
