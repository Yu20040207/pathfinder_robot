"""导航倒退许可逻辑的单测（车尾守卫是唯一的放行依据）。"""

from fwr_bringup.base_bridge_core import nav_reverse_ok


def test_forward_never_needs_the_guard():
    assert nav_reverse_ok(0.10, True, True) is True
    assert nav_reverse_ok(0.10, False, False) is True


def test_reverse_allowed_only_when_rear_is_clear():
    assert nav_reverse_ok(-0.10, False, True) is True
    assert nav_reverse_ok(-0.10, True, True) is False


def test_reverse_refused_when_guard_disabled():
    """守卫关掉 = 回到车尾盲区，不允许倒车。"""
    assert nav_reverse_ok(-0.10, False, False) is False
