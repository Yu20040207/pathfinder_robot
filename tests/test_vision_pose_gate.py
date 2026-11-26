"""姿态二次校验（2026-11-26）：杂物别当人，但低机位只看得到腿也要认得出人。

实测依据（本机抓 6 帧 + 带人样图裁腿）：
* 蓝筐上的塑料袋：检测分 0.10~0.27（阈值 0.35 就在旁边 → "有时"误报），
  姿态框分 **0.003**，但它旁边会出现 **0.9 分的关键点** → 只看关键点数会被骗；
* 只露下半身的人：姿态框分 **0.73**，髋/膝/踝 4 个可见；
* 近处腿特写：姿态框分 **0.58**，膝/踝 2~4 个。
"""

from __future__ import annotations

import numpy as np

from fwr_bringup.vision_boxes import (LEG_KEYPOINTS, filter_persons,
                                      parse_pose_output, pose_confirms)


def _kp(**kw):
    """构造 17 关键点置信度；kw 用编号。"""
    out = [0.0] * 17
    for k, v in kw.items():
        out[int(k[1:])] = v
    return out


def _pose_row(box, score, kp):
    return {'box': box, 'score': score, 'kp': kp}


def test_leg_keypoints_definition():
    assert set(LEG_KEYPOINTS) == {11, 12, 13, 14, 15, 16}


def test_legs_only_person_passes_without_shoulders():
    """低机位只看得到腿：没有肩，也必须过。"""
    row = _pose_row((100, 200, 80, 200), 0.58,
                    _kp(k13=0.65, k14=0.70, k15=0.80, k16=0.82))
    assert pose_confirms((105, 205, 80, 195), [row]) is True


def test_full_body_person_passes():
    row = _pose_row((50, 50, 120, 300), 0.89,
                    _kp(k5=0.99, k6=0.99, k11=0.98, k12=0.97, k13=0.9, k15=0.9))
    assert pose_confirms((52, 52, 118, 296), [row]) is True


def test_clutter_with_high_keypoints_but_tiny_box_score_is_rejected():
    """塑料袋：关键点 0.9 分、但姿态框分 0.003 → 不许过。"""
    row = _pose_row((140, 120, 120, 90), 0.003,
                    _kp(k5=0.93, k11=0.9, k13=0.88, k15=0.9))
    assert pose_confirms((140, 120, 120, 90), [row]) is False


def test_no_pose_row_is_rejected():
    assert pose_confirms((10, 10, 50, 100), []) is False


def test_pose_box_without_leg_keypoints_is_rejected():
    """有姿态框但只有上半身关键点（比如挂在墙上的衣服）→ 拒。"""
    row = _pose_row((10, 10, 60, 120), 0.45, _kp(k5=0.8, k6=0.8))
    assert pose_confirms((10, 10, 60, 120), [row]) is False


def test_high_detection_score_bypasses_pose():
    """检测分 ≥ 0.70 的真人直接放行，不因姿态没跟上被丢。"""
    boxes = [(10, 10, 50, 120, 0.86)]
    assert filter_persons(boxes, []) == boxes            # 没姿态模型：原样
    weak_pose = [_pose_row((10, 10, 50, 120), 0.10, [0.0] * 17)]
    assert filter_persons(boxes, weak_pose) == boxes     # 有姿态但不过：仍放行


def test_mid_score_needs_pose():
    boxes = [(10, 10, 50, 120, 0.50), (300, 10, 60, 130, 0.50)]
    good = _pose_row((10, 10, 50, 120), 0.6, _kp(k13=0.7, k15=0.7))
    kept = filter_persons(boxes, [good])
    assert kept == [boxes[0]]


def test_empty_pose_rows_keeps_old_behaviour():
    boxes = [(1, 2, 3, 4, 0.40)]
    assert filter_persons(boxes, []) == boxes


def test_parse_pose_output_shapes():
    # (1, 56, N)：4 框 + 1 分 + 17*3
    n = 3
    arr = np.zeros((1, 56, n), np.float32)
    arr[0, :4, 0] = [240, 240, 120, 240]     # cx,cy,w,h（480 输入）
    arr[0, 4, 0] = 0.7
    arr[0, 5 + 2::3, 0] = 0.2
    arr[0, 5 + 2 + 13 * 3, 0] = 0.8          # 膝 13
    rows = parse_pose_output(arr, 640, 480, 480)
    assert len(rows) == 1
    r = rows[0]
    assert abs(r['score'] - 0.7) < 1e-6
    s = 640 / 480                            # 输入 480 → 画面 640 的缩放
    assert abs(r['box'][0] - (240 - 60) * s) < 1e-6
    assert abs(r['box'][2] - 120 * s) < 1e-6
    assert r['kp'][13] > 0.5


def test_parse_pose_output_drops_low_score():
    arr = np.zeros((1, 56, 2), np.float32)
    arr[0, 4, :] = [0.01, 0.02]
    assert parse_pose_output(arr, 640, 480, 480) == []
