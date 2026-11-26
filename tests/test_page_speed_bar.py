"""跟随画面下方的"当前速度"条（2026-11-26 用户要求）。

需求原话：「现在在画面的下面显示当前速度」，
随后补充：**「手动模式里面不用显示，只显示在跟随模式就好」** → 手动/导航都不要。

* 只在 `#followStage` 之后出现（画面下方），不是压在画面上；
* 数据来源是车端上行状态里**已有的** `speed`（里程计实测 m/s）和 `yaw_rate`（rad/s），
  不新增 MQTT 字段、不改车端；
* 手动模式、导航摄像头画面里**不许**有（用户明确要求，有单测钉死）。
"""

from __future__ import annotations

import pathlib

PAGE = pathlib.Path(__file__).resolve().parents[1] / 'cloud/web/car.html'
SRC = PAGE.read_text(encoding='utf-8')


def test_bar_only_in_follow_mode():
    assert SRC.count('class="spdBar"') == 1          # 只有跟随那一条
    assert 'id="spdBarFollow"' in SRC
    assert 'id="spdBarManual"' not in SRC            # 用户：手动模式不用显示
    assert 'id="spdBarNav"' not in SRC               # 导航画面也别加回来


def test_bar_sits_below_the_follow_stage_not_over_it():
    bar = SRC.index('id="spdBarFollow"')
    assert bar > SRC.index('id="followStage"')
    assert bar < SRC.index('id="followJoyWrap"')     # 画面下面、摇杆上面
    css = SRC[SRC.index('.spdBar {'):]
    css = css[:css.index('}')]
    assert 'position: absolute' not in css           # 不是浮层
    assert 'background: #0d1117' in css              # 直角深底，和页面其它条一致


def test_uses_existing_state_fields():
    fn = SRC[SRC.index('function updateSpdBar('):]
    fn = fn[:fn.index('\n}\n')]
    assert 'm/s' in fn and 'rad/s' in fn
    assert "$('spdF')" in fn and "$('spdWF')" in fn
    assert "updateSpdBar(s.speed, s.yaw_rate)" in SRC


def test_manual_view_has_no_bar():
    """手动模式那块的 DOM 里不能出现速度条（`#camPane` 到 `#joyWrap` 之间）。"""
    seg = SRC[SRC.index('id="camPane"'):SRC.index('<div class="joyWrap">')]
    assert 'spdBar' not in seg
