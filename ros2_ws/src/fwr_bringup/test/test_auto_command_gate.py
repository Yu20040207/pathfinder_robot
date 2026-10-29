from geometry_msgs.msg import Twist
from fwr_bringup.auto_command_gate import AutoCommandGate


def test_state_transition_invalidates_command_and_timestamp_together():
    from geometry_msgs.msg import Twist
    gate = AutoCommandGate()
    cmd = Twist()
    cmd.linear.x = .06
    gate.observe_state('stopped', 0.)
    gate.observe_state('navigating', .01)
    gate.observe_nav(cmd, .02)
    gate.observe_state('selecting', .03)
    gate.observe_state('navigating', .04)
    assert gate.select(None, None, None, .05).linear.x == 0.
from fwr_bringup.base_bridge_core import RecoveryPermitCore


def cmd(v, w=0):
    msg = Twist()
    msg.linear.x, msg.angular.z = float(v), float(w)
    return msg


def test_default_zero_stale_and_exclusive_recovery():
    g = AutoCommandGate()
    assert g.select(cmd(.06), cmd(-.06), 'stopped', 0).linear.x == 0
    g.observe_state('navigating', 1.)
    g.observe_nav(cmd(.06), 1.)
    assert g.select(None, None, None, 1.05).linear.x == .06
    g.observe_state('reversing', 1.1)
    assert g.select(None, None, None, 1.11).linear.x == 0
    g.observe_recovery(cmd(-.06), 1.12)
    assert g.select(None, None, None, 1.13).linear.x == -.06
    assert g.select(None, None, None, 1.5).linear.x == 0
    g.manual_takeover()
    assert g.select(None, None, None, 1.14).linear.x == 0


def test_permit_denies_reverse_until_explicit_episode_and_budget():
    p = RecoveryPermitCore()
    assert not p.accept_reverse(-.06, 0.)
    assert not p.request(True, 1, 0., healthy=False, nav_inactive=True, manual_off=True)[0]
    assert p.request(True, 1, 0., healthy=True, nav_inactive=True, manual_off=True)[0]
    assert p.accept_reverse(-.06, .1)
    p.observe_distance(.11)
    assert not p.accept_reverse(-.06, .2)
    assert p.request(True, 1, .3, healthy=True, nav_inactive=True, manual_off=True)[0]
    p.observe_distance(.2)
    p.observe_distance(.1)
    assert not p.accept_reverse(-.06, .4)
    p.reset()
    assert not p.accept_reverse(-.06, .5)
    assert not p.request(True, 1, .6, healthy=True, nav_inactive=True, manual_off=True)[0]


def test_fault_or_reconnect_retires_current_episode():
    p = RecoveryPermitCore()
    assert p.request(True, 4, 0., True, True, True)[0]
    p.abort()
    assert not p.request(True, 4, .1, True, True, True)[0]
    assert p.request(True, 5, .2, True, True, True)[0]


def test_manual_takeover_latches_until_new_explicit_session():
    g = AutoCommandGate()
    g.observe_state('navigating', 1.)
    g.observe_nav(cmd(.06), 1.)
    assert g.select(None, None, None, 1.01).linear.x > 0
    g.observe_manual(True, 1.02)
    assert g.select(None, None, None, 1.03).linear.x == 0
    g.observe_manual(False, 1.04)
    assert g.select(None, None, None, 1.05).linear.x == 0
    g.observe_state('stopped', 1.06)
    g.observe_state('selecting', 1.07)
    g.observe_nav(cmd(.06), 1.08)
    g.observe_state('navigating', 1.09)
    g.observe_nav(cmd(.06), 1.10)
    assert g.select(None, None, None, 1.11).linear.x > 0


def test_speed_scale_scales_both_axes_and_preserves_curvature():
    gate = AutoCommandGate()
    cmd = Twist()
    cmd.linear.x, cmd.angular.z = .10, .40
    gate.observe_state('navigating', 1.0)
    gate.observe_nav(cmd, 1.0)
    out = gate.select(None, None, 'navigating', 1.01)
    assert out.linear.x == .10 and out.angular.z == .40
    gate.speed_scale = .5
    out = gate.select(None, None, 'navigating', 1.02)
    assert abs(out.linear.x - .05) < 1e-9
    assert abs(out.angular.z - .20) < 1e-9
    # 曲率（w/v）保持不变，转弯半径不会被倍率改变。
    assert abs(out.angular.z / out.linear.x - .40 / .10) < 1e-9


def test_gate_passes_commands_up_to_the_current_speed_envelope():
    """硬编码 0.10 曾把 0.16 m/s 的正常指令整帧丢弃。"""
    from geometry_msgs.msg import Twist
    from fwr_bringup.auto_command_gate import AutoCommandGate
    gate = AutoCommandGate()
    cmd = Twist()
    cmd.linear.x = .16
    gate.observe_state('navigating', 1.0)
    gate.observe_nav(cmd, 1.0)
    out = gate.select(None, None, 'navigating', 1.01)
    assert abs(out.linear.x - .16) < 1e-9
    cmd.linear.x = .19
    gate.observe_nav(cmd, 1.02)
    assert abs(gate.select(None, None, 'navigating', 1.03).linear.x - .19) < 1e-9


def test_boost_scales_straight_only_not_rotation():
    """×1.25 只提高直线速度；转向保持基速，避免堵转。"""
    from geometry_msgs.msg import Twist
    from fwr_bringup.auto_command_gate import AutoCommandGate
    gate = AutoCommandGate()
    gate.speed_scale = 1.25
    cmd = Twist()
    cmd.linear.x, cmd.angular.z = .10, .8
    gate.observe_state('navigating', 1.0)
    gate.observe_nav(cmd, 1.0)
    out = gate.select(None, None, 'navigating', 1.01)
    assert abs(out.linear.x - .10 * 1.25) < 1e-9
    assert abs(out.angular.z - .8) < 1e-9
    # 降速时两者一起降
    gate.speed_scale = .75
    out = gate.select(None, None, 'navigating', 1.02)
    assert abs(out.linear.x - .10 * .75) < 1e-9
    assert abs(out.angular.z - .8 * .75) < 1e-9


def test_boosted_linear_is_clamped_below_the_wheel_ceiling():
    """0.225 m/s 会触发固件超速锁存；倍速后的线速度必须封顶 0.20。"""
    from geometry_msgs.msg import Twist
    from fwr_bringup.auto_command_gate import AutoCommandGate
    gate = AutoCommandGate()
    gate.speed_scale = 1.25
    cmd = Twist()
    cmd.linear.x = .18
    gate.observe_state('navigating', 1.0)
    gate.observe_nav(cmd, 1.0)
    assert abs(gate.select(None, None, 'navigating', 1.01).linear.x - .20) < 1e-9
