"""人体框几何 / id 维持 / YOLO 输出解析的单元测试（不需要 ROS、不需要模型）。"""

from __future__ import annotations

import numpy as np

from fwr_bringup.vision_boxes import (assign_ids, center_dist, iou,
                                     pick_selected,
                                      parse_yolo_output, to_payload)


def test_iou_basic():
    assert iou((0, 0, 10, 10), (0, 0, 10, 10)) > 0.99
    assert iou((0, 0, 10, 10), (20, 20, 10, 10)) == 0.0
    half = iou((0, 0, 10, 10), (5, 0, 10, 10))
    assert 0.3 < half < 0.4


def test_center_dist():
    assert center_dist((0, 0, 10, 10), (10, 0, 10, 10)) == 10.0


def _yolo(person_xywh=(160.0, 160.0, 60.0, 120.0), other=True, n=20):
    """构造 YOLO 形状的输出 (1, 4+C, N)，只放一个人 + 一辆车。"""
    classes = 80
    arr = np.zeros((1, 4 + classes, n), dtype=np.float32)
    x, y, w, h = person_xywh
    arr[0, 0, 0] = x
    arr[0, 1, 0] = y
    arr[0, 2, 0] = w
    arr[0, 3, 0] = h
    arr[0, 4 + 0, 0] = 0.9          # person 置信度
    if other:
        arr[0, 0, 1] = 40.0
        arr[0, 1, 1] = 40.0
        arr[0, 2, 1] = 80.0
        arr[0, 3, 1] = 60.0
        arr[0, 4 + 2, 1] = 0.95     # car（class 2）
    return arr


def test_parse_only_person_class():
    out = _yolo()
    boxes = parse_yolo_output(out, img_w=640, img_h=480, input_size=416,
                              conf=0.35, person_class=0)
    assert len(boxes) == 1
    x, y, w, h, s = boxes[0]
    # 416 输入拉伸到 640x480：x 缩放 640/416
    assert abs(x - (160 - 30) * 640 / 416) < 2
    assert abs(w - 60 * 640 / 416) < 2
    assert 0.85 < s < 0.95


def test_parse_ignores_low_conf():
    out = _yolo()
    out[0, 4, 0] = 0.10
    assert parse_yolo_output(out, 640, 480, 416, 0.35) == []


def test_parse_empty_output():
    assert parse_yolo_output(np.zeros((1, 84, 0), dtype=np.float32),
                             640, 480, 416, 0.35) == []


def test_assign_ids_keeps_id_when_box_moves():
    prev = [(1, (100, 100, 50, 100, 0.9)), (2, (300, 100, 50, 100, 0.8))]
    boxes = [(108, 104, 50, 100, 0.9), (302, 108, 50, 100, 0.8)]
    out = assign_ids(boxes, prev)
    assert [p for p, _ in out] == [1, 2]


def test_assign_ids_new_id_for_new_person():
    prev = [(1, (100, 100, 50, 100, 0.9))]
    out = assign_ids([(500, 300, 50, 100, 0.9)], prev)
    assert out[0][0] == 2


def test_assign_ids_center_fallback():
    """IoU=0 但中心很近（人走得快）时仍应保持原 id。"""
    prev = [(7, (100, 100, 40, 80, 0.9))]
    out = assign_ids([(125, 100, 40, 80, 0.9)], prev)   # 中心距 25 < 0.35*80=28
    assert out[0][0] == 7


def test_assign_ids_far_move_is_new_person():
    """IoU 很小且中心超出兜底半径 → 必须当新目标，不能沿用旧 id。"""
    prev = [(7, (100, 100, 40, 80, 0.9))]
    out = assign_ids([(160, 100, 40, 80, 0.9)], prev)   # 中心距 60 > 28
    assert out[0][0] == 8


def test_assign_ids_two_people_not_swapped():
    prev = [(1, (100, 100, 50, 100, .9)), (2, (400, 100, 50, 100, .9))]
    boxes = [(430, 105, 50, 100, .9), (105, 102, 50, 100, .9)]
    out = assign_ids(boxes, prev)
    assert [p for p, _ in out] == [2, 1]


def test_to_payload_normalized():
    out = to_payload([(3, (320.0, 240.0, 64.0, 48.0, 0.876))], 640, 480)
    assert out[0]['id'] == 3
    assert abs(out[0]['x'] - 0.5) < 1e-6
    assert abs(out[0]['w'] - 0.1) < 1e-6
    assert out[0]['s'] == 0.876


def test_pick_selected_reclaims_after_id_churn():
    """2026-11-22 用户报"选好又变白"：id 被换号时用上次的框重新认领。"""
    last = (100, 100, 50, 100, .9)          # 上一帧选中的那个人的框
    # 本帧同一个人的 id 变成了 7（位置只挪了一点）
    ids = [(7, (108, 103, 52, 100, .88)), (2, (400, 100, 50, 100, .9))]
    assert pick_selected(ids, last) == 7


def test_pick_selected_none_when_everyone_left():
    last = (100, 100, 50, 100, .9)
    ids = [(2, (500, 120, 50, 100, .9))]     # 离得太远，不是他
    assert pick_selected(ids, last) is None


def test_pick_selected_empty_inputs():
    assert pick_selected([], (1, 2, 3, 4, .5)) is None
    assert pick_selected([(1, (0, 0, 10, 10, .9))], None) is None


def test_detector_imports_every_helper_it_uses():
    """2026-11-22 事故回归：给 `_run` 加了 `pick_selected(...)` 调用，但 import 行没跟着改
    （字符串 replace 静默没匹配上）→ 一框选目标就 `NameError`，检测线程整个死掉，
    手机端显示"人体识别没在跑"。这里静态核对"调用了的 helper 必须 import 过"。
    """
    import ast
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[1]
           / 'ros2_ws/src/fwr_bringup/fwr_bringup/person_detector.py').read_text()
    tree = ast.parse(src)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == 'fwr_bringup.vision_boxes':
            imported.update(a.name for a in node.names)
    used = {n.id for n in ast.walk(tree)
            if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
    for name in ('assign_ids', 'parse_yolo_output', 'pick_selected', 'to_payload'):
        assert name in used, name
        assert name in imported, f'{name} 被调用但没有 import'
