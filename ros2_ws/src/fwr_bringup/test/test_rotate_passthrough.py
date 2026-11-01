"""纯旋转放行：车头够远时，前停区的"全零"不得否决原地转身。"""
import time

from fwr_bringup.directional_safety_core import DirectionalSafetyCore


def _core():
    c = DirectionalSafetyCore()
    c.command_timeout = 10.0
    c.sensor_timeout = 10.0
    c.reply_timeout = 10.0
    c.monitor_warmup = 0.0
    c.require_manual_status = False
    return c


def _prime(core, now=100.0):
    core.observe_explore_state("navigating", now)
    core.observe_scan(now)
    core.observe_tf(True, now)


def test_rotate_passes_when_front_is_clear():
    c = _core()
    _prime(c)
    c.observe_raw(0.0, 0.5, 100.0)          # 纯旋转
    c.rotate_clear_ok = True
    x, z = c.select(100.1)
    assert z != 0.0, '车头空旷时应放行旋转'
    assert x == 0.0


def test_rotate_blocked_when_front_is_close():
    c = _core()
    _prime(c)
    c.observe_raw(0.0, 0.5, 100.0)
    c.rotate_clear_ok = False               # 车头太近
    x, z = c.select(100.1)
    assert z == 0.0, '车头太近时不得放行旋转'


def test_translation_is_not_passed_through():
    """有线速度时不能走放行分支（前进仍受前停区约束）。"""
    c = _core()
    _prime(c)
    c.observe_raw(0.1, 0.0, 100.0)
    c.rotate_clear_ok = True
    x, z = c.select(100.1)
    assert x == 0.0, '没有监控回执时前进必须被拒'
