import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fwr_bringup.directional_safety_core import DirectionalSafetyCore


def ready(direction='forward'):
    core = DirectionalSafetyCore()
    core.observe_monitor('forward', True, .6)
    core.observe_monitor('reverse', True, .6)
    core.observe_monitor('forward', True, 1.0)
    core.observe_monitor('reverse', True, 1.0)
    core.observe_scan(1.0)
    core.observe_tf(True, 1.0)
    core.observe_manual(False, 1.0)
    core.observe_explore_state('navigating' if direction == 'forward' else 'reversing', 1.0)
    return core


def test_forward_and_reverse_require_matching_monitor_result():
    core = ready()
    core.observe_raw(.08, .02, 1.01)
    assert core.select(1.02) == (0.0, 0.0)
    core.observe_filtered('reverse', -.08, .02, 1.03)
    assert core.select(1.04) == (0.0, 0.0)
    core.observe_filtered('forward', .04, .01, 1.05)
    assert core.select(1.06) == (.04, .01)
    core.observe_explore_state('reversing', 1.07)
    core.observe_raw(-.05, .01, 1.08)
    assert core.select(1.09) == (0.0, 0.0)
    core.observe_filtered('reverse', -.04, 0.0, 1.10)
    assert core.select(1.11) == (-.04, 0.0)


def test_forward_stop_never_selects_reverse():
    core = ready()
    core.observe_raw(.08, 0.0, 1.01)
    core.observe_filtered('reverse', -.04, 0.0, 1.02)
    core.observe_filtered('forward', 0.0, 0.0, 1.03)
    assert core.select(1.04) == (0.0, 0.0)


def test_identical_repeated_raw_can_use_very_recent_monitored_result():
    core = ready()
    core.observe_raw(.10, 0.0, 1.01)
    core.observe_filtered('forward', .07, 0.0, 1.02)
    assert core.select(1.03) == (.07, 0.0)
    core.observe_raw(.10, 0.0, 1.05)
    assert core.select(1.06) == (.07, 0.0)
    assert core.select(1.16) == (.07, 0.0)  # reused inside command_timeout
    assert core.select(1.36) == (0.0, 0.0)  # bounded hold elapsed (reply_timeout .30)
    core.observe_filtered('forward', 0.0, 0.0, 1.19)
    assert core.select(1.20) == (0.0, 0.0)


def test_new_scan_allows_bounded_hold_for_a_changed_command():
    core = ready()
    core.observe_raw(.10, 0.0, 1.01)
    core.observe_filtered('forward', .07, 0.0, 1.02)
    core.observe_scan(1.04)
    core.observe_raw(.10, 0.0, 1.05)
    assert core.select(1.06) == (.07, 0.0)
    assert core.select(1.09) == (.07, 0.0)
    core.observe_filtered('forward', .07, 0.0, 1.10)
    assert core.select(1.11) == (.07, 0.0)
    # A changed but not larger command keeps the bounded hold: the monitor
    # already approved .07 m/s, which still fits inside .09 m/s.
    core.observe_raw(.09, 0.0, 1.12)
    assert core.select(1.13) == (.07, 0.0)
    # Asking for less than the monitored output still fails closed.
    core.observe_raw(.05, 0.0, 1.14)
    assert core.select(1.35) == (0.0, 0.0)


def test_smoother_jitter_does_not_halve_the_output_duty_cycle():
    """20 Hz commands + 10 Hz monitor replies must not zero every other frame."""
    core = ready()
    core.observe_raw(0.0, .30, 1.00)
    core.observe_filtered('forward', 0.0, .30, 1.00)
    core.observe_filtered('reverse', 0.0, .30, 1.00)
    for step in range(1, 11):
        now = 1.00 + .05 * step
        command = .30 + .01 * step
        core.observe_scan(now)
        core.observe_tf(True, now)
        core.observe_monitor('forward', True, now)
        core.observe_monitor('reverse', True, now)
        core.observe_explore_state('navigating', now)
        core.observe_manual(False, now)
        core.observe_raw(0.0, command, now)
        if step % 2 == 0:
            core.observe_filtered('forward', 0.0, command, now)
            core.observe_filtered('reverse', 0.0, command, now)
        assert core.select(now)[1] != 0.0, f'frame {step} was zeroed by a stale reply'


def test_monitor_stop_overrides_bounded_hold_on_same_command():
    core = ready()
    core.observe_raw(.10, 0.0, 1.01)
    core.observe_filtered('forward', .07, 0.0, 1.02)
    core.observe_scan(1.04)
    core.observe_raw(.10, 0.0, 1.05)
    assert core.select(1.06) == (.07, 0.0)
    core.observe_filtered('forward', 0.0, 0.0, 1.065)
    assert core.select(1.07) == (0.0, 0.0)
    core.observe_monitor('forward', False, 1.075)
    assert core.select(1.08) == (0.0, 0.0)


def test_unselected_monitor_result_does_not_replace_selected_result():
    core = ready()
    core.observe_raw(.08, 0.0, 1.01)
    core.observe_filtered('forward', .08, 0.0, 1.02)
    core.observe_filtered('reverse', .08, 0.0, 1.03)
    assert core.select(1.04) == (.08, 0.0)


def test_oversized_reordered_or_wrong_sign_fails_closed():
    # Oversized, wrong sign, extra turn, and non-finite replies stay closed.
    # 方向相反/非有限值仍然整帧拒绝。
    for velocity, turn, at in [(-.04, 0.0, 1.03), (math.nan, 0.0, 1.03)]:
        core = ready()
        core.observe_raw(.08, .02, 1.01)
        core.observe_filtered('forward', velocity, turn, at)
        assert core.select(1.04) == (0.0, 0.0)
    # 幅值超出只夹紧到当前指令，不再整帧丢弃（否则转弯会被砍成顿挫）。
    core = ready()
    core.observe_raw(.08, .02, 1.01)
    core.observe_filtered('forward', .09, .03, 1.03)
    assert core.select(1.04) == (.08, .02)


def test_reordered_smaller_reply_is_reused_within_the_bound():
    core = ready()
    core.observe_raw(.08, .02, 1.01)
    core.observe_filtered('forward', .08, 0.0, 1.00)
    # An older reply computed for a larger command asks for no more than the
    # current one, so bounded reuse is allowed; it is never scaled up.
    assert core.select(1.04) == (.08, 0.0)


def test_freshness_and_state_fail_closed():
    core = ready()
    core.observe_raw(.08, 0.0, 1.01)
    core.observe_filtered('forward', .08, 0.0, 1.02)
    assert core.select(1.03) == (.08, 0.0)
    assert core.select(1.30) == (0.0, 0.0)
    core.observe_scan(1.31)
    core.observe_explore_state('stopped', 1.31)
    assert core.select(1.32) == (0.0, 0.0)
    core.observe_manual(True, 1.33)
    assert core.select(1.34) == (0.0, 0.0)


def test_missing_or_stale_odom_tf_blocks_monitored_motion():
    core = ready()
    core.observe_tf(False, 1.0)
    core.observe_raw(.08, 0.0, 1.01)
    core.observe_filtered('forward', .08, 0.0, 1.02)
    assert core.select(1.03) == (0.0, 0.0)
    core.observe_tf(True, 1.03)
    assert core.select(1.04) == (.08, 0.0)
    core.observe_tf(False, 1.05)
    assert core.select(1.06) == (0.0, 0.0)
    core.observe_tf(True, 1.07)
    core.observe_monitor('forward', True, 1.27)
    core.observe_scan(1.27)
    core.observe_manual(False, 1.27)
    core.observe_explore_state('navigating', 1.27)
    core.observe_raw(.08, 0.0, 1.28)
    core.observe_filtered('forward', .08, 0.0, 1.29)
    assert core.select(1.34) == (0.0, 0.0)


def test_zero_linear_turn_requires_both_monitors():
    core = ready()
    core.observe_raw(0.0, .3, 1.01)
    core.observe_filtered('forward', 0.0, .3, 1.02)
    assert core.select(1.03) == (0.0, 0.0)
    core.observe_filtered('reverse', 0.0, .2, 1.04)
    assert core.select(1.05) == (0.0, .2)
    core.observe_raw(0.0, .3, 1.06)
    core.observe_filtered('forward', 0.0, .3, 1.07)
    core.observe_filtered('reverse', 0.0, 0.0, 1.08)
    assert core.select(1.09) == (0.0, 0.0)


def test_rotation_rejects_opposite_or_stale_monitor():
    core = ready()
    core.observe_raw(0.0, -.3, 1.01)
    core.observe_filtered('forward', 0.0, -.3, 1.02)
    core.observe_filtered('reverse', 0.0, .3, 1.03)
    assert core.select(1.04) == (0.0, 0.0)
    core.observe_filtered('reverse', 0.0, -.3, 1.05)
    assert core.select(1.06) == (0.0, -.3)
    assert core.select(1.36) == (0.0, 0.0)


def test_sim_without_bridge_manual_topic_uses_explicit_sim_mode():
    core = DirectionalSafetyCore(require_manual_status=False)
    core.observe_monitor('forward', True, .6)
    core.observe_monitor('reverse', True, .6)
    core.observe_monitor('forward', True, 1.0)
    core.observe_monitor('reverse', True, 1.0)
    core.observe_scan(1.0)
    core.observe_tf(True, 1.0)
    core.observe_explore_state('navigating', 1.0)
    core.observe_raw(.06, 0.0, 1.01)
    core.observe_filtered('forward', .06, 0.0, 1.02)
    assert core.select(1.03) == (.06, 0.0)
    core.observe_manual(True, 1.04)
    assert core.select(1.05) == (0.0, 0.0)


def test_monitor_start_and_restart_wait_for_new_scans_and_stable_active_state():
    core = DirectionalSafetyCore(require_manual_status=False)
    core.observe_tf(True, 1.0)
    core.observe_explore_state('navigating', 1.0)
    core.observe_scan(1.0)
    core.observe_raw(.08, 0.0, 1.01)
    core.observe_filtered('forward', .08, 0.0, 1.02)
    assert core.select(1.03) == (0.0, 0.0)
    core.observe_monitor('forward', True, 1.03)
    core.observe_scan(1.04)
    core.observe_raw(.08, 0.0, 1.05)
    core.observe_filtered('forward', .08, 0.0, 1.06)
    assert core.select(1.07) == (0.0, 0.0)
    core.observe_monitor('forward', True, 1.38)
    core.observe_scan(1.38)
    core.observe_explore_state('navigating', 1.38)
    core.observe_tf(True, 1.38)
    core.observe_raw(.08, 0.0, 1.39)
    core.observe_filtered('forward', .08, 0.0, 1.40)
    assert core.select(1.41) == (.08, 0.0)
    core.observe_monitor('forward', False, 1.42)
    assert core.select(1.43) == (0.0, 0.0)
    core.observe_monitor('forward', True, 1.44)
    core.observe_scan(1.45)
    core.observe_raw(.08, 0.0, 1.46)
    core.observe_filtered('forward', .08, 0.0, 1.47)
    assert core.select(1.48) == (0.0, 0.0)


def test_shim_rotation_ramp_does_not_flap_direction():
    """The shim's few-mm/s linear ramp must not flip rotate<->forward every frame."""
    core = ready()
    core.observe_raw(0.0, .30, 1.00)
    core.observe_filtered('forward', 0.0, .30, 1.00)
    core.observe_filtered('reverse', 0.0, .30, 1.00)
    assert core.select(1.00)[1] != 0.0
    for step in range(1, 11):
        now = 1.00 + .05 * step
        linear = 0.0 if step % 2 else 0.007
        angular = .30 + .005 * step
        core.observe_scan(now)
        core.observe_tf(True, now)
        core.observe_monitor('forward', True, now)
        core.observe_monitor('reverse', True, now)
        core.observe_explore_state('navigating', now)
        core.observe_manual(False, now)
        core.observe_raw(linear, angular, now)
        if step % 2 == 0:
            core.observe_filtered('forward', linear, angular, now)
            core.observe_filtered('reverse', linear, angular, now)
        assert core.select(now)[1] != 0.0, f'step {step} zeroed by direction flap'


def test_shim_rotation_ramp_up_to_45mmps_stays_rotate():
    """Shim ramps can reach ~45 mm/s; the inner wheel is still reversing there."""
    core = ready()
    core.observe_raw(0.0, .44, 1.00)
    core.observe_filtered('forward', 0.0, .44, 1.00)
    core.observe_filtered('reverse', 0.0, .44, 1.00)
    for step in range(1, 11):
        now = 1.00 + .05 * step
        linear = 0.0 if step % 2 else 0.037
        angular = .44 + .003 * step
        core.observe_scan(now)
        core.observe_tf(True, now)
        core.observe_monitor('forward', True, now)
        core.observe_monitor('reverse', True, now)
        core.observe_explore_state('navigating', now)
        core.observe_manual(False, now)
        core.observe_raw(linear, angular, now)
        if step % 2 == 0:
            core.observe_filtered('forward', linear, angular, now)
            core.observe_filtered('reverse', linear, angular, now)
        assert core.direction == 'rotate', f'step {step} misclassified as {core.direction}'
        assert core.select(now)[1] != 0.0, f'step {step} zeroed during shim ramp'


def test_forward_arc_is_not_mistaken_for_rotation():
    core = ready()
    core.observe_raw(.06, .1, 1.01)
    assert core.direction == 'forward'
