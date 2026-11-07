from pathlib import Path

from fwr_bringup.auto_mapping_mission import CompletionGate

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'launch' / 'auto_mapping_mission.launch.py'
NODE = ROOT / 'fwr_bringup' / 'auto_mapping_mission.py'


def test_completion_requires_hold_and_resets_on_transient_done():
    gate = CompletionGate(3.0)
    assert gate.update('done 95%', 10.0) is False
    assert gate.update('done 95%', 12.9) is False
    assert gate.update('done 95%', 13.0) is True

    gate = CompletionGate(3.0)
    assert gate.update('done 95%', 10.0) is False
    assert gate.update('waiting-map-growth', 11.0) is False
    assert gate.update('done 95%', 12.0) is False
    assert gate.update('done 95%', 15.0) is True


def test_launch_uses_viewpoint_mission_without_changing_manual_or_nav_defaults():
    text = LAUNCH.read_text()
    for flag in (
        "'viewpoint_planner': 'true'",
        "'viewpoint_enabled': 'true'",
        "'mapping_bt': 'true'",
        "'use_explorer': 'false'",
        "'stuck_reverser': 'false'",
        "'kickstart': 'false'",
    ):
        assert flag in text
    assert 'real_auto_mapping.launch.py' in text
    assert 'auto_mapping_mission' in text
    assert 'save_on_done' in text


def test_finalizer_never_publishes_velocity_and_saves_both_kinds():
    text = NODE.read_text()
    assert 'Twist' not in text
    assert '/cmd_vel' not in text
    assert '/mapping/save_slam' in text
    assert '/mapping/run' in text
    assert 'motion_enabled' in text and 'manual_enabled' in text


def test_time_cap_is_wired_for_small_area_trial():
    assert 'max_runtime_s' in NODE.read_text()
    assert "declare_parameter('max_runtime_s', 0.0)" in NODE.read_text()
    assert 'time_limit' in NODE.read_text()
    assert 'max_runtime_s' in LAUNCH.read_text()
