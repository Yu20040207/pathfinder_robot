#!/usr/bin/env python3
"""多模型人体检测 A/B：同一批画面跑几个 ONNX，出对比图 + 延迟表。

用途（2026-11-26）：用户报"杂物有时被识别成人"，要先看清**哪个模型、多少分**。
不需要启动 ROS、不碰车：只读车端 `http://127.0.0.1:8081/snapshot.jpg`。

用法::

    # 车端本机抓 6 帧（每 0.8 s），跑 3 个模型，出图 /tmp/yolo_ab/ab.png
    python3 tools/yolo_ab_bench.py --frames 6

    # 已有图片目录
    python3 tools/yolo_ab_bench.py --dir /tmp/shots --out /tmp/yolo_ab

模型列表用 ``--model 名字=路径[:输入尺寸]`` 追加，默认跑仓库里的候选。
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time
import urllib.request

import cv2
import numpy as np
import onnxruntime as ort


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='', help='已有图片目录（给了就不抓帧）')
    ap.add_argument('--snapshot', default='http://127.0.0.1:8081/snapshot.jpg')
    ap.add_argument('--frames', type=int, default=6)
    ap.add_argument('--gap', type=float, default=0.8)
    ap.add_argument('--out', default='/tmp/yolo_ab')
    ap.add_argument('--conf', type=float, default=0.35, help='画框阈值（也用来算检出数）')
    ap.add_argument('--model', action='append', default=[],
                    help='名字=路径[:尺寸]，可多次给')
    return ap.parse_args()


def grab(snapshot: str, n: int, gap: float, out_dir: str) -> list:
    shots = []
    for i in range(n):
        dst = os.path.join(out_dir, f'frame{i:02d}.jpg')
        with urllib.request.urlopen(snapshot, timeout=5) as r:
            data = r.read()
        if not data.startswith(b'\xff\xd8'):
            continue
        open(dst, 'wb').write(data)
        shots.append(dst)
        time.sleep(gap)
    return shots


def load(path: str, threads: int = 4):
    so = ort.SessionOptions()
    so.intra_op_num_threads = threads
    so.inter_op_num_threads = 1
    so.add_session_config_entry('session.intra_op.allow_spinning', '0')
    so.add_session_config_entry('session.inter_op.allow_spinning', '0')
    return ort.InferenceSession(path, so, providers=['CPUExecutionProvider'])


def main() -> int:
    a = parse_args()
    os.makedirs(a.out, exist_ok=True)
    shots = sorted(glob.glob(os.path.join(a.dir, '*.jpg'))) if a.dir else \
        grab(a.snapshot, a.frames, a.gap, a.out)
    if not shots:
        print('没有画面：检查 --dir 或车端 8081 /snapshot.jpg')
        return 1

    models = []
    for spec in a.model:
        name, rest = spec.split('=', 1)
        path, _, size = rest.partition(':')
        models.append((name, path, int(size or 416)))
    if not models:
        print('必须用 --model 名字=路径[:尺寸] 指定至少一个模型')
        return 1

    from fwr_bringup.vision_boxes import parse_yolo_output  # noqa: PLC0415

    hdr = f'{"模型":16s} {"尺寸":>4s} {"中位":>6s} {"最高人分":>8s} {"≥conf":>5s}'
    print(hdr)
    tiles = []
    for name, path, size in models:
        sess = load(path)
        inp = sess.get_inputs()[0].name
        times, tops, counts = [], [], []
        for f in shots:
            img = cv2.imread(f)
            blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (size, size),
                                         swapRB=True, crop=False)
            t0 = time.monotonic()
            out = sess.run(None, {inp: blob})[0]
            times.append((time.monotonic() - t0) * 1e3)
            arr = np.asarray(out)[0].T
            cls = arr[:, 4:].argmax(1)
            sc = arr[np.arange(len(arr)), 4 + cls]
            person = sc[cls == 0]
            tops.append(float(person.max()) if len(person) else 0.0)
            boxes = parse_yolo_output(out, img.shape[1], img.shape[0], size,
                                      a.conf, 0.45, 0)
            counts.append(len(boxes))
            tiles.append((f'{name}@{size}px  最高人分 {tops[-1]:.2f}  框 {len(boxes)}',
                          img.copy(), boxes))
        times.sort()
        print(f'{name:16s} {size:4d} {times[len(times)//2]:6.1f} '
              f'{max(tops):8.3f} {sum(counts):5d}')

    # 拼图：每行一个模型的一帧
    cols = max(1, min(3, len(shots)))
    tw, th = 320, 240
    rows = (len(tiles) + cols - 1) // cols
    canvas = np.full((rows * (th + 22), cols * tw, 3), 24, np.uint8)
    for i, (label, img, boxes) in enumerate(tiles):
        r, c = divmod(i, cols)
        for x, y, w, h, s in boxes:
            cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)),
                          (0, 0, 255), 2)
            cv2.putText(img, f'person {s:.2f}', (int(x), max(12, int(y) - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        th_img = cv2.resize(img, (tw, th))
        y0 = r * (th + 22)
        canvas[y0:y0 + th, c * tw:(c + 1) * tw] = th_img
        cv2.putText(canvas, label[:52], (c * tw + 4, y0 + th + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1)
    out_png = os.path.join(a.out, 'ab.png')
    cv2.imwrite(out_png, canvas)
    print('对比图:', out_png)
    print('原图目录:', a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
