"""建图模式布局：右侧竖排按键靠左、摇杆靠右（2026-11-25 用户要求）。

原来的问题：右手要同时够摇杆（居中偏下）和右侧竖排按键（📷 🧭 ＋ －），
建图时一手挡一手。现在建图模式下按钮列挪到左边、摇杆贴右边：
右手握摇杆、左手点按钮。
"""

from __future__ import annotations

import pathlib
import re

PAGE = pathlib.Path(__file__).resolve().parents[1] / 'cloud/web/car.html'


def _css(selector: str) -> str:
    """取出某个 CSS 规则块的内容（只匹配**行首**那条，避免前缀命中）。"""
    src = PAGE.read_text(encoding='utf-8')
    m = re.search(r'(?m)^[ \t]*' + re.escape(selector) + r'\s*\{([^}]*)\}', src)
    assert m, f'没找到 CSS 规则：{selector}'
    return m.group(1)


def test_map_mode_moves_button_column_left():
    body = _css('body.map-mode #zoomCtl')
    assert 'left: 10px' in body, body
    assert 'right: auto' in body, body


def test_map_mode_moves_joystick_right():
    body = _css('body.map-mode #mapJoyWrap')
    assert 'justify-content: flex-end' in body, body
    assert 'padding-right' in body, body


def test_nav_mode_keeps_buttons_on_right():
    """导航模式（无 map-mode）仍是右侧竖排——这条改动不能顺手把导航也挪了。"""
    plain = _css('#zoomCtl')
    assert 'right: 10px' in plain, plain
    assert 'flex-direction: column' in plain, plain
    src = PAGE.read_text(encoding='utf-8')
    # 只允许 body.map-mode 限定符下出现 left: 10px 覆盖
    assert not re.search(r'^\s*#zoomCtl\s*\{[^}]*left:', src, re.M)


def test_apply_mode_toggles_map_mode_class():
    src = PAGE.read_text(encoding='utf-8')
    assert "document.body.classList.toggle('map-mode', isMap)" in src
    # 必须在 applyMode 内部、紧跟 mapJoyWrap 显隐之后（位置别漂）
    idx_mode = src.index('function applyMode(')
    idx_toggle = src.index("document.body.classList.toggle('map-mode', isMap)")
    assert idx_toggle > idx_mode
    # 建图模式下摇杆仍是唯一显示的那一个（不能被 hidden 逻辑漏掉）
    assert "$('mapJoyWrap').classList.toggle('hidden', !isMap)" in src
