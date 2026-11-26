"""画面正下方的"当前速度"条（2026-11-26 用户要求）。

需求原话：「现在在画面的下面显示当前速度」。
三个有画面的模式（手动 / 跟随 / 导航摄像头）都要有，且：

* 位置在**画面下方**（`#camStage` / `#followStage` / `#navCamPane` 之后），不是压在画面上；
* 数据来源是车端上行状态里**已有的** `speed`（里程计实测 m/s）和 `yaw_rate`（rad/s），
  不新增 MQTT 字段、不改车端；
* 导航那块的速度条跟摄像头画面一起显隐。
"""

from __future__ import annotations

import pathlib
import re

PAGE = pathlib.Path(__file__).resolve().parents[1] / 'cloud/web/car.html'
SRC = PAGE.read_text(encoding='utf-8')


def test_bar_exists_under_all_three_video_panes():
    assert SRC.count('class="spdBar"') >= 2          # 手动 / 跟随
    assert 'id="spdBarNav"' in SRC                   # 导航（默认 hidden）
    for pid in ('spdM', 'spdWM', 'spdF', 'spdWF', 'spdN', 'spdWN'):
        assert f'id="{pid}"' in SRC


def test_bar_sits_below_the_stage_not_over_it():
    """必须在 stage **之后**出现（画面下方），而且不是 absolute 浮层。"""
    manual = SRC.index('id="spdBarManual"')
    assert manual > SRC.index('id="camStage"')
    assert manual < SRC.index('<div class="joyWrap">')       # 画面下面、摇杆上面
    follow = SRC.index('id="spdBarFollow"')
    assert follow > SRC.index('id="followStage"')
    assert follow < SRC.index('id="followJoyWrap"')
    css = SRC[SRC.index('.spdBar {'):]
    css = css[:css.index('}')]
    assert 'position: absolute' not in css
    assert 'background: #0d1117' in css          # 直角深底，和页面其它条一致


def test_uses_existing_state_fields():
    fn = SRC[SRC.index('function updateSpdBar('):]
    fn = fn[:fn.index('\n}\n')]
    assert 'm/s' in fn and 'rad/s' in fn
    assert "updateSpdBar(s.speed, s.yaw_rate)" in SRC


def test_nav_bar_toggles_with_the_picture():
    i = SRC.index('async function openNavCam()')
    seg = SRC[i:i + 500]
    assert "$('spdBarNav')" in seg and "classList.remove('hidden')" in seg
    j = SRC.index('function closeNavCam()')
    seg2 = SRC[j:j + 500]
    assert "$('spdBarNav')" in seg2 and "classList.add('hidden')" in seg2
