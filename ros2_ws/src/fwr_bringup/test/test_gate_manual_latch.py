"""门控手动锁存：脱困短退用完手动通道后，导航必须能恢复。"""
from geometry_msgs.msg import Twist

from fwr_bringup.auto_command_gate import AutoCommandGate


def _nav(v=0.10, w=0.0):
    cmd = Twist()
    cmd.linear.x = v
    cmd.angular.z = w
    return cmd


def test_nav_passes_when_manual_never_used():
    g = AutoCommandGate()
    g.observe_state('navigating', 0.0)
    out = g.select(_nav(), None, 'navigating', 0.05)
    assert out.linear.x > 0


def test_manual_takeover_blocks_nav():
    g = AutoCommandGate()
    g.observe_manual(True, 0.0)            # 脱困短退打开手动通道
    g.observe_state('navigating', 0.1)
    out = g.select(_nav(), None, 'navigating', 0.15)
    assert out.linear.x == 0.0


def test_latch_clears_once_manual_is_off():
    """手动关掉且模式回到 navigating → 锁存必须解除（否则车永远不动）。"""
    g = AutoCommandGate()
    g.observe_manual(True, 0.0)
    g.observe_state('navigating', 0.1)
    assert g.select(_nav(), None, 'navigating', 0.15).linear.x == 0.0

    g.observe_manual(False, 0.2)           # 短退结束
    g.observe_state('navigating', 0.3)     # 回到导航模式
    out = g.select(_nav(), None, 'navigating', 0.35)
    assert out.linear.x > 0, '导航应恢复（手动锁存已解除）'
