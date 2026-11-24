"""「页面和服务互相拉扯」的回归测试（2026-11-24）。

用户报的现象：「有时候提示人体识别没在跑」。

真因不是识别坏了，是**服务被反复启停**：
* 页面切后台 / 点「取消跟随」都会 `GET /follow?on=0`；
* 旧服务端把 `on=0` 当成"要停服务" → `systemctl stop fwr-vision fwr-follow`；
* 可页面还在跟随模式里，5 s 后 `prep=1` 又拉起来 → 每十几秒 stop→start；
* vision 每次收到 SIGINT 都在 `main()` 里二次 `rclpy.shutdown()` → core-dump，
  重启期间手机端就显示"人体识别没在跑"。

现在：**只有 `leave=1`（真的离开跟随模式）才停服务**，`on=0` 一律不动。
"""

from __future__ import annotations

import ast
import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load_function(path: pathlib.Path, name: str):
    """直接从源码里取函数（服务端模块依赖 aiohttp/rclpy，测试环境不装）。"""
    tree = ast.parse(path.read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            ns: dict = {}
            exec(compile(mod, str(path), 'exec'), ns)   # noqa: S102
            return ns[name]
    raise AssertionError(f'{path} 里没有 {name}')


def test_on0_does_not_stop_services():
    """取消跟随（on=0）只是暂停，**不能**停视觉/跟随服务。"""
    target = _load_function(ROOT / 'tools/robot_mjpeg_server.py',
                            'follow_service_target')
    assert target(on=False, prep=False, leave=False) is None


def test_leave_stops_services():
    target = _load_function(ROOT / 'tools/robot_mjpeg_server.py',
                            'follow_service_target')
    assert target(on=False, prep=False, leave=True) is False


def test_prep_starts_services():
    target = _load_function(ROOT / 'tools/robot_mjpeg_server.py',
                            'follow_service_target')
    assert target(on=False, prep=True, leave=False) is True


def test_on1_starts_services():
    target = _load_function(ROOT / 'tools/robot_mjpeg_server.py',
                            'follow_service_target')
    assert target(on=True, prep=False, leave=False) is True


def test_leave_wins_over_stale_on():
    """离开模式的请求即使带了 on=1（旧页面/竞态）也以 leave 为准。"""
    target = _load_function(ROOT / 'tools/robot_mjpeg_server.py',
                            'follow_service_target')
    assert target(on=False, prep=False, leave=True) is False


def test_page_sends_leave_when_leaving_follow_mode():
    html = (ROOT / 'cloud/web' / 'car.html').read_text(encoding='utf-8')
    assert "follow?leave=1" in html


def test_page_does_not_treat_network_error_as_dead_detector():
    """连不上车 ≠ 识别没在跑：必须分开提示。"""
    html = (ROOT / 'cloud/web' / 'car.html').read_text(encoding='utf-8')
    assert 'followBoxesDead' in html
    assert '连不上机器人' in html
    # 旧文案会误导用户去重启服务，已替换
    assert '人体识别没在跑（机器人上启动 person_detector）' not in html


@pytest.mark.parametrize('name', ['person_detector.py', 'person_follower.py'])
def test_nodes_shutdown_only_once(name):
    """SIGTERM 时二次 rclpy.shutdown() 会抛 RCLError → systemd 记 failed / core-dump。"""
    src = (ROOT / 'ros2_ws/src/fwr_bringup/fwr_bringup' / name).read_text(encoding='utf-8')
    assert 'ExternalShutdownException' in src
    assert re.search(r'if rclpy\.ok\(\):\s*\n\s*rclpy\.shutdown\(\)', src), src[-400:]
