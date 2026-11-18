"""统一让位判据：**执行前**决定"直接转"还是"先退"。

2026-11-18 用户定的规则（现场原话）：
    "监测到快碰撞的时候直接后退就可以了，后退到能转弯的过去了就好，
     还有种情况也就是左右两边都很接近，这样也是直接后退就好，
     就是在执行前就判断好是后退还是直接转弯就好。"

这里既测单个函数，也把**用户的两个现场场景**串成完整过程跑一遍：
触发 → 后退 → 退到又能转了立刻停。
"""
import pytest

from fwr_bringup.maneuver_core import (accumulate_blocked, backup_done,
                                       can_turn, lateral_ok, wants_backup)


# ---------- 基础判据 ----------

def test_lateral_ok_needs_both_sides():
    assert lateral_ok(0.50, 0.52, 0.45)
    assert not lateral_ok(0.50, 0.30, 0.45)   # 一侧窄就不行


def test_can_turn_needs_front_too():
    # 两侧够但车头贴墙 → 不能就地转
    assert not can_turn(0.60, 0.60, 0.20)
    assert can_turn(0.60, 0.60, 0.60)
    assert can_turn(0.46, 0.46, 0.46)


# ---------- 要不要退 ----------

BASE = dict(want_turn=False, blocked_for_s=0.0, front_m=1.0,
            lat_left_m=0.70, lat_right_m=0.70,
            rear_min_m=1.20, rear_blocked=False,
            attempts=0, since_last_s=99.0)


def test_no_backup_when_can_turn_directly():
    """能转就直接转 —— 不许后退打扰 Nav2。"""
    ok, why = wants_backup(**{**BASE, 'want_turn': True})
    assert not ok and why == 'can-turn'


def test_backup_when_turning_but_sides_tight():
    """用户场景二：左右两边都很接近 → 直接后退。"""
    ok, why = wants_backup(**{**BASE, 'want_turn': True,
                              'lat_left_m': 0.32, 'lat_right_m': 0.28})
    assert ok and why == 'turn-too-tight'


def test_backup_when_front_about_to_hit():
    """用户场景一：快撞了 → 直接后退（不需要等真的被压零很久）。"""
    ok, why = wants_backup(**{**BASE, 'want_turn': False,
                              'front_m': 0.20, 'blocked_for_s': 0.6})
    assert ok and why == 'front-about-to-hit'


def test_no_backup_when_front_close_but_not_blocked_long():
    ok, why = wants_backup(**{**BASE, 'front_m': 0.20, 'blocked_for_s': 0.2})
    assert not ok and why == 'no-need'


def test_no_backup_without_rear_room():
    """车尾不够就别退，退了只会贴上后墙。"""
    ok, why = wants_backup(**{**BASE, 'want_turn': True,
                              'lat_left_m': 0.3, 'lat_right_m': 0.3,
                              'rear_min_m': 0.40})
    assert not ok and why == 'rear-tight'
    ok, why = wants_backup(**{**BASE, 'want_turn': True,
                              'lat_left_m': 0.3, 'lat_right_m': 0.3,
                              'rear_blocked': True})
    assert not ok and why == 'rear-blocked'


def test_no_backup_when_attempts_or_cooldown():
    tight = {'want_turn': True, 'lat_left_m': 0.3, 'lat_right_m': 0.3}
    ok, why = wants_backup(**{**BASE, **tight, 'attempts': 3})
    assert not ok and why == 'attempts-exhausted'
    ok, why = wants_backup(**{**BASE, **tight, 'since_last_s': 1.0})
    assert not ok and why == 'cooldown'


# ---------- 什么时候停 ----------

DONE = dict(backed_m=0.0, elapsed_s=0.0, lat_left_m=0.30, lat_right_m=0.28,
            front_m=0.30, rear_min_m=1.20, rear_blocked=False)


def test_stops_as_soon_as_turning_is_possible():
    """核心要求："后退到能转弯的过去了就好" —— 一能转就立刻停。"""
    assert not backup_done(**DONE)[0]
    ok, why = backup_done(**{**DONE, 'backed_m': 0.35,
                             'lat_left_m': 0.50, 'lat_right_m': 0.47,
                             'front_m': 0.60})
    assert ok and why == 'can-turn-now'


def test_stops_on_distance_rear_and_timeout():
    ok, why = backup_done(**{**DONE, 'backed_m': 0.80})
    assert ok and why == 'reached'
    ok, why = backup_done(**{**DONE, 'rear_min_m': 0.15})
    assert ok and why == 'rear-close'
    ok, why = backup_done(**{**DONE, 'elapsed_s': 12.0})
    assert ok and why == 'timeout'


def test_done_and_trigger_use_same_measure():
    """触发和停止必须同源：不能出现"触发时算窄、停止时算宽"。"""
    tight = dict(front_m=0.30, lat_left_m=0.30, lat_right_m=0.28)
    assert wants_backup(**{**BASE, 'want_turn': True, **tight})[0]
    # 同样的量 → 不能同时判成"可以停了"
    assert not backup_done(**{**DONE, **tight})[0]


# ---------- 用户的两个现场场景（全过程） ----------

def _simulate(steps, **kw):
    """按给定净空序列模拟一次"触发→后退→停止"。返回 (触发原因, 停止原因, 退了多少)。"""
    trig = wants_backup(**kw)
    if not trig[0]:
        return None, None, 0.0
    backed, elapsed = 0.0, 0.0
    for lat_l, lat_r, front in steps:
        elapsed += 0.5
        backed += 0.10
        done, why = backup_done(
            backed_m=backed, elapsed_s=elapsed,
            lat_left_m=lat_l, lat_right_m=lat_r, front_m=front,
            rear_min_m=kw['rear_min_m'], rear_blocked=kw['rear_blocked'])
        if done:
            return trig[1], why, backed
    return trig[1], None, backed


def test_scenario_dead_end_backs_out_until_turn_possible():
    """现场 A（2026-11-18 16:35）：死胡同里想掉头，两侧 0.41/0.30，车尾 1.4 m。

    正常应该是：想转但转不开 → 后退 → 退到左右都 >=0.45 立刻停 → 交给 Nav2 转。
    """
    kw = dict(want_turn=True, blocked_for_s=0.0, front_m=0.23,
              lat_left_m=0.41, lat_right_m=0.30,
              rear_min_m=1.43, rear_blocked=False,
              attempts=0, since_last_s=99.0)
    steps = [(0.41, 0.30, 0.30),      # 刚退一点，仍不够
             (0.44, 0.31, 0.40),
             (0.50, 0.46, 0.55)]      # 够宽了 → 必须在这里停
    trig, stop, backed = _simulate(steps, **kw)
    assert trig == 'turn-too-tight'
    assert stop == 'can-turn-now'
    assert backed == pytest.approx(0.30), '不该退过能转的位置'


def test_scenario_both_sides_tight_backs_out():
    """现场 B：左右两边都贴墙（0.32/0.28），前方其实开着 → 也要直接后退。"""
    kw = dict(want_turn=True, blocked_for_s=0.0, front_m=0.90,
              lat_left_m=0.32, lat_right_m=0.28,
              rear_min_m=1.20, rear_blocked=False,
              attempts=0, since_last_s=99.0)
    steps = [(0.32, 0.28, 0.95), (0.38, 0.33, 1.00), (0.49, 0.46, 1.05)]
    trig, stop, backed = _simulate(steps, **kw)
    assert trig == 'turn-too-tight'
    assert stop == 'can-turn-now'
    assert backed > 0.0


def test_scenario_open_room_never_backs_up():
    """空间够的时候一次后退都不该发生。"""
    kw = dict(want_turn=True, blocked_for_s=0.0, front_m=0.80,
              lat_left_m=0.70, lat_right_m=0.66,
              rear_min_m=1.20, rear_blocked=False,
              attempts=0, since_last_s=99.0)
    trig, _, backed = _simulate([], **kw)
    assert trig is None and backed == 0.0


# ---------- 抖动容忍 ----------

def test_blocked_accumulates_through_toggling():
    acc = 0.0
    for i in range(20):
        acc = accumulate_blocked(acc, 0.1, i % 2 == 0)   # stop/release 交替
    assert acc >= 0.5


def test_blocked_decays_only_after_grace():
    acc = 1.0
    acc = accumulate_blocked(acc, 0.2, False, unblocked_s=0.2)   # 噪声，不衰减
    assert acc == 1.0
    acc = accumulate_blocked(acc, 0.2, False, unblocked_s=1.0)   # 真的通了 → 2 倍速
    assert acc == pytest.approx(0.6)


# ---------- 被挡要原地转时，往宽的那侧转（2026-11-18 用户提问） ----------

from fwr_bringup.directional_safety_core import choose_rotate_side  # noqa: E402


def test_turns_toward_wider_side_when_blocked():
    """前方被挡、要原地转出去时，明显更宽的一侧优先。

    原来是"只有请求侧 <0.16 m 才翻"，等于永不翻转 —— 死胡同里 Nav2 想往窄的
    那侧掉头，车就顶着窄墙转。
    """
    # Nav2 想往右（requested=右 0.30），左边 0.60 更宽 → 应翻到左
    assert choose_rotate_side(True, 0.30, 0.60) == 'flip'
    # 反过来
    assert choose_rotate_side(True, 0.60, 0.30) == 'same'


def test_no_flip_without_margin():
    """两边差不多宽时不要来回翻（差值 <= 0.15 视作一样）。"""
    assert choose_rotate_side(True, 0.50, 0.60) == 'same'
    assert choose_rotate_side(True, 0.50, 0.52) == 'same'


def test_front_clear_keeps_nav_direction():
    """前方没被挡时不许改方向 —— 正常行驶必须听 Nav2 的。"""
    assert choose_rotate_side(False, 0.20, 0.90) == 'same'
    # 请求侧看起来窄但前方没挡，也不该翻
    assert choose_rotate_side(False, 0.10, 0.90) == 'flip'   # 请求侧 < clear 才翻
    assert choose_rotate_side(False, 0.30, 0.90) == 'same'


def test_both_sides_tight_cannot_turn():
    """两侧都不够宽 → 不允许转（交给后退让位）。"""
    assert choose_rotate_side(True, 0.10, 0.12) is None


def test_no_pointless_backup_when_one_side_is_tight():
    """现场 2026-11-18 17:06：左 1.14 / 右 0.43 → 不该退。

    用户批准的改动：turn_need_m 0.45 → 0.40（车角扫掠 0.35 + 5 cm）。
    0.45 时这里会退 3 cm、停 2~3 s，纯属白折腾。
    """
    ok, why = wants_backup(want_turn=True, blocked_for_s=0.0, front_m=0.90,
                           lat_left_m=1.14, lat_right_m=0.43,
                           rear_min_m=2.13, rear_blocked=False,
                           attempts=0, since_last_s=99.0)
    assert not ok and why == 'can-turn'


def test_still_backs_up_when_both_sides_under_040():
    """两侧都 <0.40 才是真的转不开 → 必须后退（用户原话）。"""
    ok, why = wants_backup(want_turn=True, blocked_for_s=0.0, front_m=0.90,
                           lat_left_m=0.39, lat_right_m=0.36,
                           rear_min_m=1.20, rear_blocked=False,
                           attempts=0, since_last_s=99.0)
    assert ok and why == 'turn-too-tight'


def test_rotation_requires_both_sides():
    """原地转必须两侧都够宽 —— 左 0.67 / 右 0.16 绝对不能转。

    2026-11-18 现场："还左转撞到了障碍物"。左转时右后角会扫向右侧那堵 0.16 m
    的墙（车角扫掠半径 0.35 m），只看请求侧在几何上就是错的。
    """
    from fwr_bringup.directional_safety_core import rotation_allowed  # noqa: E402
    assert not rotation_allowed(0.67, 0.16, 0.38)
    assert not rotation_allowed(0.16, 0.67, 0.38)
    assert rotation_allowed(0.40, 0.41, 0.38)
    assert not rotation_allowed(0.37, 0.90, 0.38)


def test_selector_gates_rotation_on_both_sides():
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' /
           'directional_safety_selector.py').read_text()
    assert 'rotation_allowed(' in src
    assert "rotate_clear_m', 0.38" in src, '0.16 只够正前方，侧向必撞'
    assert 'both_ok and self._front_min_m >= clear' in src
