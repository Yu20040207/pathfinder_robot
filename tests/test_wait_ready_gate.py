"""建图模式 slam_toolbox 的启动改成"扫描就绪就起"（2026-11-24）。

原来 `real_mapping.launch.py` 用 `TimerAction(period=35.0)` 硬延后，原因是栈刚起时
后雷达/scan_merger 还没数据，SLAM 拿半圈首帧就把地图窗口定死（车不动时地图永远缺一半）。
实测双雷达 5~8 s 就稳，白等 27 s。现在 `tools/wait_ready.py` 盯扫描频率，
达标后用 OnProcessExit 起 slam_toolbox。
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (ROOT / 'tools' / 'wait_ready.py').read_text(encoding='utf-8')
LAUNCH = (ROOT / 'ros2_ws' / 'src' / 'fwr_bringup' / 'launch'
          / 'real_mapping.launch.py').read_text(encoding='utf-8')
NAV_SCRIPT = (ROOT / 'scripts' / 'run_navigation.sh').read_text(encoding='utf-8')


def test_gate_measures_real_rate():
    assert '--min-hz' in SCRIPT and '--window' in SCRIPT
    assert 'len(stamps) / args.window >= args.min_hz' in SCRIPT
    assert 'filled = now - t0 >= args.window' in SCRIPT, '窗口没填满不能算频率'


def test_gate_fails_soft():
    # 超时只报错、退出码 1；调用方照样能把 slam 起来（缺数据也比永远不开始强）
    assert 'return 0 if ok else 1' in SCRIPT
    assert '继续启动但可能缺数据' in SCRIPT


def test_mapping_launch_uses_gate_not_timer():
    assert 'TimerAction' not in LAUNCH, '建图栈里不该再有固定延时'
    assert 'wait_ready.py' in LAUNCH
    assert 'OnProcessExit(target_action=slam_gate' in LAUNCH, '就绪后再起 slam_toolbox'
    assert "'--min-hz', '8'" in LAUNCH


def test_cleanup_skipped_when_no_old_stack():
    # 没有旧栈时不做 4 个 ros2 CLI + sleep 8/2（实测白花 ~28 s）
    assert '没有旧栈在跑，跳过清理' in NAV_SCRIPT
    assert 'pgrep -f "[f]wr_bringup/base_bridge" >/dev/null 2>&1; then\n  echo "== 没有旧栈' in NAV_SCRIPT
    # 固定 sleep 8 改成轮询
    assert 'sleep 8\n  # 2026-11-12' not in NAV_SCRIPT
    assert 'for _ in $(seq 1 16); do' in NAV_SCRIPT


def test_serial_port_check_runs_on_both_paths():
    # 跳过清理时也要确认 ESP32 串口没被幽灵句柄占住
    idx_if = NAV_SCRIPT.index('没有旧栈在跑，跳过清理')
    idx_esp = NAV_SCRIPT.index('ESP32 串口仍被占用')
    assert idx_esp > idx_if
    assert NAV_SCRIPT.count('ESP32 串口仍被占用') == 1, '检查只留一处'
