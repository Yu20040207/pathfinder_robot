"""保存地图的"作用域"：建图模式保存**不抢占**当前导航地图。

2026-11-24 现场（用户报「重新定位匹配错了地图」）：
  建图模式里建完图、保存 → 保存动作顺手把 `~/.config/fwr/last_navigation_map`
  改写成了这张新图 → 切回导航时 map_server 加载的是**新图**，而手机上的图面还是
  上一张导航图（地图图片只在 set_map 时才上传）→ 定位结果画在另一张图上，
  看着就像"重新定位匹配错了地图"。

现在：建图模式只写文件、不动指针；导航模式里的地图编辑才更新指针。

另外钉死 `auto_initial_pose._on_map` 的掩码失效：换**同尺寸**的另一张图也必须清，
否则会拿旧地图的墙面掩码去匹配新图。
"""
from __future__ import annotations

import base64
import gzip
import json
from types import SimpleNamespace

import numpy as np
import pytest

from fwr_bringup import mqtt_bridge as mb
from fwr_bringup import auto_initial_pose as aip


class _Logger:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def info(self, msg, **_kw) -> None:
        self.lines.append(str(msg))

    def warn(self, msg, **_kw) -> None:
        self.lines.append(str(msg))

    def error(self, msg, **_kw) -> None:
        self.lines.append(str(msg))


def _bridge(tmp_path, *, mapping_mode: bool, mode: str):
    """不跑 rclpy 初始化，只装出 _on_map_edit 需要的那几个属性。"""
    node = mb.MqttBridge.__new__(mb.MqttBridge)
    node._mapping_mode = mapping_mode
    node._current_mode = mode
    node.MAPS_ROOT = str(tmp_path / 'maps')
    node._map_name = ''
    node._map_name_at = 0.0
    log = _Logger()
    node.get_logger = lambda: log
    results = []
    node._publish_result = lambda *a, **k: results.append((a, k))
    node._publish_map_list = lambda: None
    return node, log, results


def _map_edit_msg(name: str, width: int = 4, height: int = 3) -> SimpleNamespace:
    raw = bytes([254]) * (width * height)
    payload = {
        'name': name,
        'width': width,
        'height': height,
        'resolution': 0.05,
        'origin': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'pgm': base64.b64encode(gzip.compress(raw, 6)).decode('ascii'),
        'gz': True,
    }
    return SimpleNamespace(payload=json.dumps(payload).encode('utf-8'))


def test_mapping_mode_save_does_not_hijack_nav_map(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    state = tmp_path / '.config' / 'fwr' / 'last_navigation_map'
    state.parent.mkdir(parents=True)
    old_map = tmp_path / 'maps' / '1A_map_door2.yaml'
    old_map.parent.mkdir(parents=True)
    old_map.write_text('image: 1A_map_door2.pgm\n', encoding='utf-8')
    state.write_text(str(old_map), encoding='utf-8')

    node, log, results = _bridge(tmp_path, mapping_mode=True, mode='map')
    node._on_map_edit(_map_edit_msg('fresh_after_mapping'))

    # 文件确实写下来了（地图列表里能看到）
    assert (tmp_path / 'maps' / 'phone' / 'fresh_after_mapping.pgm').is_file()
    # 但当前导航地图**没变** —— 这正是用户要求的行为
    assert state.read_text(encoding='utf-8').strip() == str(old_map)
    assert any('不改当前导航地图' in line for line in log.lines)
    assert results and results[0][0][0] == 'nav' and results[0][0][1] is True
    assert '当前导航地图未变' in results[0][0][2]


def test_nav_mode_save_updates_nav_map(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    state = tmp_path / '.config' / 'fwr' / 'last_navigation_map'
    state.parent.mkdir(parents=True)
    state.write_text(str(tmp_path / 'old.yaml'), encoding='utf-8')

    node, log, results = _bridge(tmp_path, mapping_mode=False, mode='nav')
    node._on_map_edit(_map_edit_msg('edited_in_nav'))

    saved = tmp_path / 'maps' / 'phone' / 'edited_in_nav.yaml'
    assert saved.is_file()
    assert state.read_text(encoding='utf-8').strip() == str(saved)
    assert results and results[0][0][1] is True


def test_on_map_clears_mask_even_for_same_size_map():
    """同尺寸换图（现场都是 517x487）也必须让掩码失效。"""
    node = aip.AutoInitialPose.__new__(aip.AutoInitialPose)
    node._map = None
    node._mask_cache = np.zeros((3, 4), dtype=bool)
    node.get_logger = lambda: _Logger()

    info = SimpleNamespace(width=4, height=3, resolution=0.05)
    node._on_map(SimpleNamespace(info=info))
    assert node._mask_cache is None

    # 再换一张**同尺寸**的图：仍然必须清空
    node._mask_cache = np.ones((3, 4), dtype=bool)
    node._on_map(SimpleNamespace(info=SimpleNamespace(width=4, height=3,
                                                      resolution=0.05)))
    assert node._mask_cache is None


def test_bridge_source_scope_guards():
    """静态回归：这两个分支别被后来的重构删掉。"""
    src = (mb.__file__ and open(mb.__file__, encoding='utf-8').read()) or ''
    assert "self._mapping_mode or self._current_mode == 'map'" in src
    assert '不改当前导航地图' in src
    # 切回导航 / 导航栈重启后要把真正加载的图同步给云端
    assert src.count('_publish_map_file(self._active_map_path)') >= 2
    assert '_init_active_map()' in src
