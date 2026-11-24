"""碰撞监控改成"TF 就绪就激活"（2026-11-24，替代写死的 40 s 延迟）。

背景：40 s 是 2026-11-19 为了绕开 extrapolation（监控 active 但不发布、车停
2 分 8 秒）定下的；实测 TF 在 launch 后 10~15 s 就稳，白等 25 s。
现在 `tools/tf_gate_activate.py` 等 `laser_front→base_footprint` 在 now-1.0s
可查且连续稳定 2 s，再 configure+activate。
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (ROOT / 'tools' / 'tf_gate_activate.py').read_text(encoding='utf-8')
LAUNCH = (ROOT / 'ros2_ws' / 'src' / 'fwr_bringup' / 'launch'
          / 'real_navigation.launch.py').read_text(encoding='utf-8')


def test_script_gates_on_tf_history():
    assert '--history-s' in SCRIPT
    assert 'lookup_transform(' in SCRIPT, '必须真的查一次历史时刻的变换'
    assert 'Duration(seconds=args.history_s)' in SCRIPT
    assert 'steady_s' in SCRIPT, '要连续稳定几秒，不能查到一次就激活'


def test_script_fails_soft_for_watchdog_fallback():
    # 超时只报错退出，不阻塞 launch；看门狗仍会兜底补激活
    assert '交给 chain_watchdog 兜底' in SCRIPT
    assert 'return 1' in SCRIPT


def test_launch_uses_gate_not_fixed_delay():
    assert "_tf_gate()" in LAUNCH
    assert 'tf_gate_activate.py' in LAUNCH
    assert 'TimerAction(period=40.0' not in LAUNCH, '不能再靠 40 秒定时'
    assert "'--source-frame', 'laser_front'" in LAUNCH
    assert "'--target-frame', 'base_footprint'" in LAUNCH


def test_monitor_still_respawns():
    # 进程挂了要能自己回来，激活交给看门狗兜底
    assert 'respawn=True, respawn_delay=3.0' in LAUNCH
