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


# COCO-17 关键点里"腿部"那几个：11/12 髋、13/14 膝、15/16 踝。
# 2026-11-26 用户指出摄像头现在装在低处、靠近时只看得到腿 —— 姿态校验**不能要求肩部**。
LEG_KEYPOINTS = (11, 12, 13, 14, 15, 16)


def parse_pose_output(out, img_w: int, img_h: int, input_size: int,
                      score_min: float = 0.05) -> list:
    """YOLO-pose 输出 (1, 4+1+17*3, N) → [{box, score, kp}]（原图像素坐标）。

    ``kp`` 是 17 个关键点置信度（0~1），坐标不再解析（只用来判"是不是人"）。
    """
    if out is None or len(out) == 0:
        return []
    arr = np.asarray(out)
    pred = arr[0].T if arr.ndim == 3 else arr.T
    if pred.shape[1] < 5 + 51:
        return []
    rows = []
    sx = img_w / float(input_size)
    sy = img_h / float(input_size)
    for r in pred:
        score = float(r[4])
        if score < score_min:
            continue
        cx, cy, bw, bh = [float(v) for v in r[:4]]
        rows.append({
            'box': (max(0.0, (cx - bw / 2) * sx), max(0.0, (cy - bh / 2) * sy),
                    bw * sx, bh * sy),
            'score': score,
            'kp': [float(v) for v in r[5 + 2::3]],
        })
    return rows


def _iou(a, b) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1, y1 = max(ax, bx), max(ay, by)
    x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    if x2 <= x1 or y2 <= y1:
        return 0.0
    inter = (x2 - x1) * (y2 - y1)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


def pose_confirms(box, pose_rows, pose_conf: float = 0.30,
                  min_leg_kp: int = 1, kp_conf: float = 0.40,
                  iou_thr: float = 0.30) -> bool:
    """这个检测框是不是"真的人"——用姿态结果二次确认（腿友好）。

    规则（2026-11-26 实测定版）：
      1. 找一个和它重叠的姿态框（IoU ≥ ``iou_thr``）；
      2. **姿态框分 ≥ ``pose_conf``** —— 这一条最关键：杂物（塑料袋）会出现
         0.9 分的"关键点"，但**姿态框分只有 0.003**，只看关键点数会被骗；
      3. 髋/膝/踝里至少 ``min_leg_kp`` 个可信（≥ ``kp_conf``）—— 低机位只看得到腿时
         肩部本来就没有，所以**不要求上半身**。
    """
    best = None
    for r in pose_rows or []:
        iou = _iou(box, r['box'])
        if iou >= iou_thr and (best is None or iou > best[0]):
            best = (iou, r)
    if best is None:
        return False
    r = best[1]
    if float(r['score']) < float(pose_conf):
        return False
    kp = r['kp']
    legs = sum(1 for i in LEG_KEYPOINTS if i < len(kp) and kp[i] >= kp_conf)
    return legs >= int(min_leg_kp)


def filter_persons(boxes, pose_rows, bypass_conf: float = 0.70,
                   pose_conf: float = 0.30, min_leg_kp: int = 1,
                   kp_conf: float = 0.40, iou_thr: float = 0.30) -> list:
    """检测框 → 过一遍姿态校验。

    * 检测分 ≥ ``bypass_conf`` 直接放行（真人高置信，不因姿态没跟上被丢）；
    * 其余必须有姿态背书，否则丢掉（塑料袋/纸箱/椅背这种"长得像人"的杂物）。
    ``pose_rows`` 为空（姿态模型没装/没跑）时**原样返回**，保持老行为。
    """
    if not pose_rows:
        return list(boxes)
    out = []
    for b in boxes:
        score = float(b[4]) if len(b) > 4 else 0.0
        if score >= bypass_conf:
            out.append(b)
            continue
        if pose_confirms(tuple(b[:4]), pose_rows, pose_conf, min_leg_kp,
                         kp_conf, iou_thr):
            out.append(b)
    return out


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

# ---------------------------------------------------------------- 衣服颜色指纹
# 2026-11-26 用户报「另一个人朝我走过来，车跑去跟他了」：id 靠几何贪心匹配，
# 一换人就会把"最近的那个人"接上。颜色是最便宜的外观特征（人 2 m 内、
# 光照不变时很稳），拿它当**第二道门槛**：认领的人衣服颜色对不上就不认。
COLOR_BINS = 12          # 色相分 12 档（30° 一档）
COLOR_S_MIN = 40         # 饱和度低于这个当"没颜色"（白/灰墙、地砖）不计票
COLOR_V_MIN = 40         # 太暗的像素也不计票


def _roi_band(box, h_frac=(0.20, 0.80)) -> tuple:
    """取框中间那一段（跳过头部和脚）。

    摄像头装得低时画面里只有腿，取整条腿；全身时避开头发/脸的肤色干扰。
    """
    x, y, w, h = float(box[0]), float(box[1]), float(box[2]), float(box[3])
    y0 = int(max(0.0, y + h * h_frac[0]))
    y1 = int(y + h * h_frac[1])
    revive = 1 if w < 2 or h < 2 else 0
    return int(max(0.0, x)), y0, int(x + w), y1, revive


def color_descriptor(img, box, bins: int = COLOR_BINS):
    """框内的"衣服颜色"直方图（HSV 色相直方图，L2 归一化）。

    返回 ``numpy float32[bins]``；像素太少 / 全灰白时返回 ``None``
    （宁可判"没有颜色信息"，也不要拿噪声当指纹）。
    """
    try:
        import numpy as np
        import cv2
    except Exception:                       # pragma: no cover - 环境缺失时降级
        return None
    h_img, w_img = img.shape[:2]
    x0, y0, x1, y1, bad = _roi_band(box)
    if bad:
        return None
    x0, x1 = max(0, min(x0, w_img - 1)), max(0, min(x1, w_img))
    y0, y1 = max(0, min(y0, h_img - 1)), max(0, min(y1, h_img))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    roi = img[y0:y1, x0:x1]
    if roi.size == 0:
        return None
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hh = hsv[:, :, 0].astype(np.int32) * bins // 180
    ss = hsv[:, :, 1]
    vv = hsv[:, :, 2]
    mask = (ss >= COLOR_S_MIN) & (vv >= COLOR_V_MIN)
    total = int(mask.sum())
    if total < 0.10 * mask.size or total < 12:
        return None
    hist = np.bincount(hh[mask].ravel(), minlength=bins).astype(np.float32)
    norm = float(np.linalg.norm(hist))
    if norm <= 0.0:
        return None
    return hist / norm


def color_sim(a, b) -> float:
    """两个归一化直方图的**交集**（0~1，1 = 完全一样）。"""
    if a is None or b is None:
        return 0.0
    try:
        import numpy as np
    except Exception:                       # pragma: no cover
        return 0.0
    a = np.asarray(a, dtype=np.float32)
    b = np.asarray(b, dtype=np.float32)
    if a.shape != b.shape or a.size == 0:
        return 0.0
    return float(np.minimum(a, b).sum())


def color_accept(ref, cand, min_sim: float = 0.45) -> bool:
    """颜色是否够像。**任一缺失就放行**（宁可认错人，也不能因为天黑不认人）。"""
    if ref is None or cand is None or min_sim <= 0:
        return True
    return color_sim(ref, cand) >= float(min_sim)


def color_blend(ref, new, alpha: float = 0.25):
    """目标颜色缓慢更新（光照/距离变化），别一帧就换。"""
    if new is None:
        return ref
    if ref is None:
        return new
    try:
        import numpy as np
    except Exception:                       # pragma: no cover
        return ref
    a = max(0.0, min(1.0, float(alpha)))
    out = (1.0 - a) * np.asarray(ref, dtype=np.float32) + a * np.asarray(new, dtype=np.float32)
    norm = float(np.linalg.norm(out))
    return out / norm if norm > 0 else ref


def adopt_allowed(ref, cand, ref_age_s: float, hold_s: float = 12.0,
                  min_sim: float = 0.45) -> bool:
    """要不要把"刚出现的这个人"接成目标（外观门槛）。

    * `ref` 是**当前目标**的衣服颜色指纹，`ref_age_s` 是它最后一次更新的时间差；
    * 超过 `hold_s`（12 s）没更新就过期：光照/角度可能已经变了，不再拦（放行）；
    * 指纹缺失（天黑、纯灰白、框太小）一律放行 —— 宁可认错，也不要在暗处不认人；
    * 否则要求 `color_sim >= min_sim`。
    """
    if ref is None or min_sim <= 0:
        return True
    if float(ref_age_s) > float(hold_s):
        return True
    if cand is None:
        return True
    return color_sim(ref, cand) >= float(min_sim)
