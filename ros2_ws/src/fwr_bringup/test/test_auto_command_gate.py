from geometry_msgs.msg import Twist
from fwr_bringup.auto_command_gate import AutoCommandGate
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
