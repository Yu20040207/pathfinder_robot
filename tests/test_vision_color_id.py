"""衣服颜色指纹（2026-11-26 用户报「另一个人朝我走过来，车跑去跟他了」）。

`assign_ids` 是几何贪心匹配，`pick_selected` 按"上次的框"就近认领，
`person_detector` 还有一条「画面里只剩一个人就认他」的规则 —— 三条都只看位置，
所以**路人走到车前面就会被当成目标**。颜色是最便宜的外观特征，

这里分成两层，各自可测：
* `color_descriptor()` / `color_sim()`：从检测框里取 HSV 色相直方图当指纹；
* `adopt_allowed()`：认领前的外观门槛（指纹过期/缺失一律放行，暗处不能不认人）。

外加 detector 侧的"目标丢了 3 s 才允许独占画面认领"（`adopt_lone_min_s`）。
"""

from __future__ import annotations

import numpy as np

from fwr_bringup.vision_boxes import (adopt_allowed, color_accept,
                                      color_blend, color_descriptor, color_sim)

BOX = (50, 40, 100, 120)


def _solid(bgr=(0, 0, 200), h=200, w=200):
    img = np.zeros((h, w, 3), np.uint8)
    img[:, :] = bgr
    return img


def test_same_color_is_similar():
    red = _solid((0, 0, 200))
    a = color_descriptor(red, BOX)
    b = color_descriptor(_solid((0, 0, 190)), BOX)
    assert a is not None and b is not None
    assert color_sim(a, b) > 0.9
    assert color_accept(a, b, 0.45)


def test_different_color_is_rejected():
    red = color_descriptor(_solid((0, 0, 200)), BOX)
    blue = color_descriptor(_solid((200, 0, 0)), BOX)
    assert color_sim(red, blue) < 0.2
    assert not color_accept(red, blue, 0.45)


def test_grey_or_dark_has_no_descriptor():
    """灰白墙、全黑：没有颜色信息 → 返回 None（门槛随后自动放行）。"""
    assert color_descriptor(_solid((128, 128, 128)), BOX) is None
    assert color_descriptor(_solid((10, 10, 10)), BOX) is None


def test_descriptor_is_normalised():
    d = color_descriptor(_solid((0, 0, 200)), BOX)
    assert abs(float(np.linalg.norm(d)) - 1.0) < 1e-5


def test_box_outside_image_is_safe():
    img = _solid()
    # 完全在图外 → 没有像素可用
    assert color_descriptor(img, (-50, -50, 20, 20)) is None
    # 只露出一条边（裁完不足 4 像素）→ 也不硬算
    assert color_descriptor(img, (195, 195, 40, 40)) is None
    # 一半在图内 → 正常出指纹
    assert color_descriptor(img, (150, 150, 100, 100)) is not None


def test_adopt_allowed_rules():
    red = color_descriptor(_solid((0, 0, 200)), BOX)
    blue = color_descriptor(_solid((200, 0, 0)), BOX)
    # 指纹新鲜 + 颜色不像 → 不许认
    assert not adopt_allowed(red, blue, ref_age_s=1.0, hold_s=12.0, min_sim=0.45)
    # 指纹新鲜 + 颜色像 → 认
    assert adopt_allowed(red, red, ref_age_s=1.0, hold_s=12.0, min_sim=0.45)
    # 指纹过期（光照/角度早就变了）→ 放行
    assert adopt_allowed(red, blue, ref_age_s=20.0, hold_s=12.0, min_sim=0.45)
    # 没有指纹 / 候选人取不到颜色 → 放行（宁可认错也不能在暗处不认人）
    assert adopt_allowed(None, blue, ref_age_s=1.0)
    assert adopt_allowed(red, None, ref_age_s=1.0)
    # 门槛设 0 = 关断
    assert adopt_allowed(red, blue, ref_age_s=1.0, min_sim=0.0)


def test_blend_keeps_norm_and_slides_slowly():
    red = color_descriptor(_solid((0, 0, 200)), BOX)
    blue = color_descriptor(_solid((200, 0, 0)), BOX)
    mixed = color_blend(red, blue, 0.25)
    assert abs(float(np.linalg.norm(mixed)) - 1.0) < 1e-5
    # α=0.25 只挪一点点：和 red 的相似度必须仍然高于 blue
    assert color_sim(mixed, red) > color_sim(mixed, blue)


def test_detector_requires_three_seconds_before_lone_adoption():
    """「只剩一个人就认他」必须等到目标丢了 3 s，不能路人一进来就认。"""
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[1]
           / 'ros2_ws/src/fwr_bringup/fwr_bringup/person_detector.py').read_text(encoding='utf-8')
    assert "adopt_lone_min_s', 3.0" in src
    assert 'self._sel_miss >= self.adopt_lone_min_s' in src
    assert '_adopt_color_ok' in src and 'color_ref' in src
