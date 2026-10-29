from pathlib import Path
import math
import sys
from fwr_bringup.auto_mapping_preflight import PreflightSnapshot, evaluate_preflight

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
from check_auto_mapping_sim import (front_stop_acceptance, is_armed_motion_state,
                                    rear_block_acceptance, reverse_acceptance)


def safe_snapshot():
    return PreflightSnapshot(scan_age=.1, mapping_scan_age=.1, mapping_health_age=.1,
            mapping_healthy=True, mapping_publisher_count=1, mapping_publisher_is_filter=True,
            imu_age=.1, odom_age=.1, map_age=.3, base_state_age=.1,
            tf_ok=True, monitor_forward_active=True, monitor_reverse_active=True,
            selector_present=True, safe_publisher_count=1, safe_publisher_is_selector=True,
            nav_ready=True, fault=0,
            keyboard_running=False, manual_enabled=False, motion_enabled=False,
            measured_length_m=.59, measured_width_m=.42, stop_distance_m=.20)


def test_stale_scan_monitor_fault_keyboard_all_reject():
    for override in ({'scan_age': .5}, {'base_state_age': .5}, {'monitor_reverse_active': False},
                     {'fault': 1}, {'keyboard_running': True}):
        snapshot = safe_snapshot()
        snapshot.__dict__.update(override)
        assert not evaluate_preflight(snapshot)[0]
    assert evaluate_preflight(safe_snapshot())[0]


def test_bad_footprint_or_stopping_distance_rejects():
    s = safe_snapshot()
    s.measured_width_m = .45
    assert not evaluate_preflight(s)[0]
    for field, value in [('measured_length_m', 0), ('measured_width_m', -1),
                         ('measured_width_m', math.nan), ('stop_distance_m', math.nan)]:
        s = safe_snapshot()
        setattr(s, field, value)
        assert not evaluate_preflight(s)[0]
    s = safe_snapshot()
    s.stop_distance_m = .35
    assert not evaluate_preflight(s)[0]


def test_shell_exit_closes_gate_and_no_auto_resume():
    text = (ROOT / 'scripts' / 'start_auto_mapping.sh').read_text()
    assert 'trap cleanup EXIT' in text
    assert 'trap' in text and 'INT' in text and 'TERM' in text
    assert '/explore/stop' in text
    assert 'motion_enabled false' in text
    assert 'map_saver_cli' in text
    assert 'auto_mapping_preflight' in text
    assert '/explore/start' in text
    assert '/base/arm_auto' in text


def test_reverse_acceptance_requires_real_bounded_motion_and_stop():
    assert reverse_acceptance(8, 16, 4, .21, .099, True)
    assert not reverse_acceptance(8, 16, 0, 0., 0., True)
    assert not reverse_acceptance(8, 16, 4, 0., 0., True)
    assert not reverse_acceptance(8, 16, 4, .31, .099, True)
    assert not reverse_acceptance(8, 16, 4, .21, .12, True)
    assert not reverse_acceptance(8, 16, 4, .21, .099, False)


def test_reverse_command_is_armed_motion_in_probe():
    assert is_armed_motion_state('navigating')
    assert is_armed_motion_state('reversing')
    assert not is_armed_motion_state('stopped')


def test_front_stop_evidence_ignores_later_reverse_motion():
    assert front_stop_acceptance(injected_at=10., last_forward_at=10.1, now=12.)
    assert not front_stop_acceptance(injected_at=10., last_forward_at=11.5, now=12.)
    assert not front_stop_acceptance(injected_at=10., last_forward_at=8., now=12.)


def test_rear_block_requires_attempt_but_no_safe_reverse_or_motion():
    assert rear_block_acceptance(8, 0, 0.)
    assert not rear_block_acceptance(0, 0, 0.)
    assert not rear_block_acceptance(8, 1, 0.)
    assert not rear_block_acceptance(8, 0, .01)
