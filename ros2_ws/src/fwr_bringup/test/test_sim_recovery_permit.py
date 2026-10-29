from geometry_msgs.msg import Twist
from fwr_bringup.base_bridge_core import RecoveryPermitCore
from fwr_bringup.sim_recovery_permit import select_sim_safe_command


def command(v):
    msg = Twist()
    msg.linear.x = v
    return msg


def test_reverse_requires_live_permit_and_budget():
    core = RecoveryPermitCore()
    assert select_sim_safe_command(command(-.06), 'reversing', True, core, 1.).linear.x == 0
    assert core.request(True, 1, 1., True, True, True)[0]
    assert select_sim_safe_command(command(-.06), 'reversing', True, core, 1.1).linear.x < 0
    core.observe_distance(.101)
    assert select_sim_safe_command(command(-.06), 'reversing', True, core, 1.2).linear.x == 0
    assert select_sim_safe_command(command(.06), 'navigating', False, core, 1.3).linear.x == 0


def test_zero_between_reverse_packets_does_not_consume_permit():
    core = RecoveryPermitCore()
    assert core.request(True, 1, 1., True, True, True)[0]
    assert select_sim_safe_command(command(0.0), 'reversing', True, core, 1.05).linear.x == 0
    assert core.active
    assert select_sim_safe_command(command(-.06), 'reversing', True, core, 1.10).linear.x == -.06
