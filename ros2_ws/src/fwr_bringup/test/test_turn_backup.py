"""转弯让位：要转大角度但车头贴太近 → 直线后退一点腾空间。

2026-11-18 现场：车要掉头，前进被 front_stop 压零 → 改成原地转又被压零
→ 车停住 → 规划进度超时 → 目标失败。用户要求"后退一点让出位置"。
"""
from fwr_bringup.turn_backup_core import backup_done, wants_backup

BASE = dict(blocked_for_s=1.5, raw_linear=0.05, raw_angular=0.55,
            front_min_m=0.32, rear_min_m=0.80, rear_blocked=False,
            attempts=0, since_last_s=10.0)


def test_backs_up_when_front_close_and_turning():
    ok, why = wants_backup(**BASE)
    assert ok and why == 'turn-blocked-front-close'


def test_no_backup_for_straight_motion():
    """想直走被挡是死胡同那套逻辑管的，这里不能抢。"""
    ok, why = wants_backup(**{**BASE, 'raw_angular': 0.02})
    assert not ok and why == 'not-turn-intent'


def test_no_backup_when_front_is_far():
    """车头不近就后退也解决不了（可能是侧向被卡）。"""
    ok, why = wants_backup(**{**BASE, 'front_min_m': 1.2})
    assert not ok and why == 'front-not-close'


def test_no_backup_when_rear_blocked():
    ok, why = wants_backup(**{**BASE, 'rear_blocked': True})
    assert not ok and why == 'rear-blocked'


def test_no_backup_when_rear_tight():
    ok, why = wants_backup(**{**BASE, 'rear_min_m': 0.2})
    assert not ok and why == 'rear-tight'


def test_attempt_limit_and_cooldown():
    ok, _ = wants_backup(**{**BASE, 'attempts': 3})
    assert not ok, '不能无限倒退'
    ok, why = wants_backup(**{**BASE, 'since_last_s': 0.5})
    assert not ok and why == 'cooldown'


def test_backup_stop_conditions():
    assert backup_done(0.05, 0.5, 1.0, False)[1] == ''
    assert backup_done(0.25, 0.5, 1.0, False) == (True, 'reached')
    assert backup_done(0.05, 9.0, 1.0, False) == (True, 'timeout')
    assert backup_done(0.05, 0.2, 1.0, True) == (True, 'rear-blocked')
    assert backup_done(0.05, 0.2, 0.10, False) == (True, 'rear-close')


def test_distance_limit_is_short():
    """后退只为腾出转弯空间，不该退太多（默认 0.20 m）。"""
    import inspect
    sig = inspect.signature(backup_done)
    assert sig.parameters['dist_m'].default <= 0.30
