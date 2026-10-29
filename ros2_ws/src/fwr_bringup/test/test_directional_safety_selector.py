from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_selector_topics_and_timer_are_wired():
    source = (ROOT / 'fwr_bringup' / 'directional_safety_selector.py').read_text()
    assert "'/cmd_vel_auto_unchecked'" in source
    assert "'/cmd_vel_auto_forward_filtered'" in source
    assert "'/cmd_vel_auto_reverse_filtered'" in source
    assert "'/cmd_vel_auto_safe'" in source
    assert "'/explore/state'" in source
    assert "'/base/manual_enabled'" in source
    assert "'scan_topic'" in source
    assert 'create_timer(.05' in source


def test_selector_entry_point_is_registered():
    setup = (ROOT / 'setup.py').read_text()
    assert 'directional_safety_selector = fwr_bringup.directional_safety_selector:main' in setup


def test_selector_shutdown_checks_context_before_publishing():
    source = (ROOT / 'fwr_bringup' / 'directional_safety_selector.py').read_text()
    assert 'if rclpy.ok():\n            node.publisher.publish(Twist())' in source


def test_sim_selector_checks_odom_to_its_configured_base_frame():
    from types import SimpleNamespace
    from fwr_bringup.directional_safety_selector import DirectionalSafetySelector

    calls = []

    class Buffer:
        def lookup_transform(self, target, source, _):
            calls.append((target, source))
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)))

    fake = SimpleNamespace(
        base_frame='base_link', tf_buffer=Buffer(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)))
    assert DirectionalSafetySelector._tf_ready(fake)
    assert calls == [('odom', 'base_link'), ('base_link', 'laser_front')]
    launch = (ROOT / 'launch' / 'corridor_sim.launch.py').read_text()
    assert "'base_frame': 'base_link'" in launch
