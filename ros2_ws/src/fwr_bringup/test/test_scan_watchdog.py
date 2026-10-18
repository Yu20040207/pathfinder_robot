from fwr_bringup.base_bridge_core import ScanWatchdog


def test_scan_missing_or_stale_stops_motion():
    gate = ScanWatchdog(timeout_s=.25)
    assert not gate.ready(0.0)
    assert not gate.observe([], 0.01)
    assert not gate.ready(.02)
    assert gate.observe([float('inf'), .7], .03)
    assert gate.ready(.20)
    assert not gate.ready(.29)


def test_scan_invalid_ranges_do_not_refresh():
    gate = ScanWatchdog()
    assert not gate.observe([float('nan'), float('inf'), -1], 1.0)
    assert not gate.ready(1.01)
    assert gate.observe([.5], 1.1)
    assert gate.ready(1.2)
    assert not gate.observe([float('inf')], 1.21)
    assert not gate.ready(1.4)


def test_sensor_stamp_replay_delay_and_future_are_rejected():
    from fwr_bringup.base_bridge_core import SensorStampGuard
    guard = SensorStampGuard(timeout_s=.25)
    assert guard.accept(100.0, 100.02, 1.0)
    assert not guard.accept(100.0, 100.1, 1.1)
    assert not guard.accept(100.1, 100.5, 1.5)
    assert not guard.accept(101.0, 100.5, 1.6)
    assert not guard.ready(1.3)
    assert guard.accept(100.51, 100.52, 1.7)
    assert guard.ready(1.8)


def test_nearly_expired_scan_does_not_gain_full_arrival_timeout():
    from fwr_bringup.base_bridge_core import SensorStampGuard
    guard = SensorStampGuard(timeout_s=.25)
    assert guard.accept(100.0, 100.24, 1.0)
    assert guard.ready(1.0)
    assert not guard.ready(1.02)
