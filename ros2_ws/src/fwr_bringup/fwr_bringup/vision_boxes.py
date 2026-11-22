"""人体框的纯逻辑（不 import rclpy / onnxruntime，方便单元测试）。

`person_detector` 节点只做「取帧 → 推理 → 调这里」，把容易写错的几何和 id 维持
放在这个模块里，`tests/test_person_detector.py` 直接测它。
"""

from __future__ import annotations

import cv2
import numpy as np


def iou(a, b) -> float:
    """两个 (x, y, w, h) 框的 IoU。"""
    ax, ay, aw, ah = a[:4]
    bx, by, bw, bh = b[:4]
    x1 = max(ax, bx)
    y1 = max(ay, by)
    x2 = min(ax + aw, bx + bw)
    y2 = min(ay + ah, by + bh)
    if x2 <= x1 or y2 <= y1:
        return 0.0
    inter = (x2 - x1) * (y2 - y1)
    return float(inter / (aw * ah + bw * bh - inter + 1e-9))


def center_dist(a, b) -> float:
    """两个框中心的距离（像素）。"""
    return float(np.hypot((a[0] + a[2] / 2) - (b[0] + b[2] / 2),
                          (a[1] + a[3] / 2) - (b[1] + b[3] / 2)))


def parse_yolo_output(out, img_w: int, img_h: int, input_size: int,
                      conf: float, iou_thr: float = 0.45,
                      person_class: int = 0) -> list:
    """YOLOv8/11 输出 (1, 4+classes, N) → [(x, y, w, h, score)]（原图像素）。"""
    if out is None or len(out) == 0:
        return []
    arr = np.asarray(out)
    pred = arr[0].T if arr.ndim == 3 else arr.T      # (N, 4+classes)
    if pred.shape[1] < 5:
        return []
    scores = pred[:, 4:]
    cls = scores.argmax(axis=1)
    sc = scores[np.arange(len(scores)), cls]
    keep = (cls == person_class) & (sc >= conf)
    if not keep.any():
        return []
    p = pred[keep]
    c = sc[keep]
    cx, cy, bw, bh = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
    boxes = np.stack([cx - bw / 2, cy - bh / 2, bw, bh], axis=1)
    boxes = boxes * np.array([img_w / input_size, img_h / input_size,
                              img_w / input_size, img_h / input_size])
    rects = boxes.astype(int).tolist()
    idx = cv2.dnn.NMSBoxes(rects, c.astype(float).tolist(), conf, iou_thr)
    out_boxes = []
    for i in (idx if len(idx) else []):
        i = int(i)
        x, y, bw2, bh2 = [float(v) for v in rects[i]]
        x = max(0.0, min(x, img_w - 2))
        y = max(0.0, min(y, img_h - 2))
        bw2 = max(8.0, min(bw2, img_w - x))
        bh2 = max(8.0, min(bh2, img_h - y))
        out_boxes.append((x, y, bw2, bh2, float(c[i])))
    return out_boxes


def assign_ids(boxes, prev: list) -> list:
    """把本帧检测框和上一帧的 id 做贪心匹配（IoU 优先，中心距兜底）。

    `prev` 形如 ``[(id, (x, y, w, h)), ...]``，返回同样的形状。
    兜底匹配是为了「人走得快、两帧 IoU 掉到 0」时 id 不要跳、选中的目标别丢。
    """
    result: list = []
    used: set = set()
    next_id = max([pid for pid, _ in prev], default=0) + 1
    for box in boxes:
        best, best_score = None, 0.0
        for i, (pid, pbox) in enumerate(prev):
            if i in used:
                continue
            score = iou(box, pbox)
            if score < 0.2:
                # IoU 不够就只认「中心够近」，否则一律当新目标：
                # 之前把「IoU 0.14」也算匹配，两个挨着的人会互相抢 id。
                reach = 0.35 * max(pbox[2], pbox[3])
                d = center_dist(box, pbox)
                score = 0.19 * (1 - d / reach) if d < reach else 0.0
            if score > best_score:
                best, best_score = i, score
        if best is None:
            result.append((next_id, box))
            next_id += 1
        else:
            used.add(best)
            result.append((prev[best][0], box))
    return result


def pick_selected(ids, box_px, gate_ratio: float = 0.6,
                  iou_min: float = 0.05):
    """在本帧检测框里**重新认领**上次选中的那个人（2026-11-22）。

    背景：选中目标靠 `assign_ids` 维持 id，人走得快 / 尺度变化 / 短暂遮挡时
    贪心匹配会失败，id 被换成新号 → 页面按 id 比颜色，框立刻变白，
    手机上只能重新点一次。这里用"上次的框"再找一遍最近的那个人，
    找到就把 id 接上（调用方负责把 sel_id 改成新 id）。

    判据放宽到 IoU 0.05，否则按中心距：``d < 0.6·max(w,h)``。
    只有一个候选时调用方可以直接认（画面里就一个人）。
    """
    if not ids or box_px is None:
        return None
    bw, bh = float(box_px[2]), float(box_px[3])
    reach = gate_ratio * max(bw, bh)
    best, best_score = None, 0.0
    for pid, box in ids:
        score = iou(box, box_px)
        if score < iou_min:
            d = center_dist(box, box_px)
            score = (0.04 * (1.0 - d / reach)) if (reach > 0 and d < reach) else 0.0
        if score > best_score:
            best, best_score = pid, score
    return best


def to_payload(ids, img_w: int, img_h: int) -> list:
    """[(id, (x,y,w,h,score))] → 归一化的 JSON 列表（手机端按 0~1 画框）。"""
    out = []
    for pid, b in ids:
        out.append({'id': int(pid),
                    'x': round(float(b[0]) / img_w, 4),
                    'y': round(float(b[1]) / img_h, 4),
                    'w': round(float(b[2]) / img_w, 4),
                    'h': round(float(b[3]) / img_h, 4),
                    's': round(float(b[4]) if len(b) > 4 else 0.0, 3)})
    return out
