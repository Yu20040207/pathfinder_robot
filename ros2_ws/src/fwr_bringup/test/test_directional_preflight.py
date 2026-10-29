import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fwr_bringup.auto_mapping_preflight import PreflightSnapshot, evaluate_preflight


def nominal():
    return PreflightSnapshot(scan_age=0.01, mapping_scan_age=0.01,
                             mapping_health_age=0.01, mapping_healthy=True,
                             mapping_publisher_count=1, mapping_publisher_is_filter=True,
                             imu_age=0.01, odom_age=0.01,
                             map_age=0.01, base_state_age=0.01, tf_ok=True,
                             monitor_forward_active=True, monitor_reverse_active=True,
                             selector_present=True, safe_publisher_count=1,
                             safe_publisher_is_selector=True,
                             nav_ready=True, fault=0, keyboard_running=False,
                             manual_enabled=False, motion_enabled=False,
                             measured_length_m=.5, measured_width_m=.35,
                             stop_distance_m=.2)


def test_both_monitors_and_sole_selector_are_required():
    assert evaluate_preflight(nominal())[0]
    for field, value in [('monitor_forward_active', False),
                         ('monitor_reverse_active', False),
                         ('selector_present', False),
                         ('safe_publisher_count', 0),
                         ('safe_publisher_count', 2),
                         ('safe_publisher_is_selector', False)]:
        snapshot = nominal()
        setattr(snapshot, field, value)
        assert not evaluate_preflight(snapshot)[0], field


def test_filtered_scan_and_filter_health_are_required():
    assert evaluate_preflight(nominal())[0]
    for field, value in [('mapping_scan_age', 1.0),
                         ('mapping_health_age', 1.0),
                         ('mapping_healthy', False),
                         ('mapping_publisher_count', 0),
                         ('mapping_publisher_count', 2),
                         ('mapping_publisher_is_filter', False)]:
        snapshot = nominal()
        setattr(snapshot, field, value)
        assert not evaluate_preflight(snapshot)[0], field
