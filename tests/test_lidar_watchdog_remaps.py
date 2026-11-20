"""看门狗兜底启动的 remaps 解包（2026-11-20 崩溃回归）。

历史 bug：`SPECS` 里 rear 的 remaps 写成扁平 `('/scan', '/scan_rear')`，
`for src, dst in state.remaps` 把字符串当序列解包 →
`ValueError: too many values to unpack (expected 2)`，看门狗进程崩、
systemd 反复重启（当天计数 67），期间两个雷达都没人管。

这里用轻量 rclpy / sensor_msgs 桩件导入工具脚本，只测纯函数，不启动 ROS。
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys
import types

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'lidar_watchdog.py'


def _install_stubs() -> None:
    if 'rclpy' in sys.modules:
        return

    class _Node:                                  # noqa: D401 - 仅作基类
        def __init__(self, *args, **kwargs) -> None:
            pass

    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    executors = types.ModuleType('rclpy.executors')
    executors.MultiThreadedExecutor = type('MultiThreadedExecutor', (), {})
    node = types.ModuleType('rclpy.node')
    node.Node = _Node
    qos = types.ModuleType('rclpy.qos')
    qos.qos_profile_sensor_data = object()
    rclpy.executors = executors
    rclpy.node = node
    rclpy.qos = qos

    sensor_msgs = types.ModuleType('sensor_msgs')
    msg = types.ModuleType('sensor_msgs.msg')
    msg.LaserScan = type('LaserScan', (), {})
    sensor_msgs.msg = msg

    if 'rcl_interfaces' not in sys.modules:
        rcl = types.ModuleType('rcl_interfaces')
        srv = types.ModuleType('rcl_interfaces.srv')
        rcl.srv = srv
        sys.modules['rcl_interfaces'] = rcl
        sys.modules['rcl_interfaces.srv'] = srv

    sys.modules.update({
        'rclpy': rclpy,
        'rclpy.executors': executors,
        'rclpy.node': node,
        'rclpy.qos': qos,
        'sensor_msgs': sensor_msgs,
        'sensor_msgs.msg': msg,
    })


def _load():
    _install_stubs()
    spec = importlib.util.spec_from_file_location('lidar_watchdog_under_test', TOOL)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_specs_remaps_expand_to_pairs():
    module = _load()
    for spec in module.SPECS:
        args = module._remap_args(spec[5])
        for src, dst in args:                     # 不能抛 ValueError
            assert src.startswith('/') and dst.startswith('/')


def test_rear_remap_is_one_pair():
    module = _load()
    assert module._remap_args((('/scan', '/scan_rear'),)) == [('/scan', '/scan_rear')]


def test_flat_tuple_still_accepted():
    module = _load()
    assert module._remap_args(('/scan', '/scan_rear')) == [('/scan', '/scan_rear')]


def test_none_and_garbage_are_safe():
    module = _load()
    assert module._remap_args(None) == []
    assert module._remap_args(()) == []
    assert module._remap_args(('single',)) == []
