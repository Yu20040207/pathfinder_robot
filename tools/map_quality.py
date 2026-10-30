#!/usr/bin/env python3
"""地图质量评分：把"这张图好不好"变成可比较的数字。

指标
  1. walls            墙格数量
  2. free_m2          空闲面积
  3. usable_m2        可放目标面积（空闲区腐蚀 0.30 m 后的最大连通块）
  4. dirs             主墙方向（Hough 峰值），用来判断房间是否被画歪
  5. spread_deg       主方向组的最大分散度（越小越直）
  6. pairs            互相垂直的方向对数（房间应该有 2 组正交方向）

用法：
  .venv/bin/python tools/map_quality.py 地图.yaml [地图2.yaml ...]
"""
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def load_map(yaml_path: Path):
    meta = yaml.safe_load(yaml_path.read_text())
    image = yaml_path.parent / meta['image']
    arr = np.array(Image.open(image))
    res = float(meta.get('resolution', 0.05))
    return arr, res


def dominant_dirs(occ: np.ndarray, top: int = 4) -> list[float]:
    ys, xs = np.nonzero(occ)
    if len(ys) < 100:
        return []
    pts = np.stack([xs, ys], 1).astype(float)
    scores = []
    for deg in np.arange(0.0, 180.0, 0.5):
        th = np.radians(deg)
        proj = pts[:, 0] * np.cos(th) + pts[:, 1] * np.sin(th)
        hist, _ = np.histogram(proj, bins=int((proj.max() - proj.min()) / 1.5) + 2)
        scores.append((deg, float((hist.astype(float) ** 2).sum())))
    scores.sort(key=lambda x: -x[1])
    peaks: list[float] = []
    for deg, _ in scores:
        if all(min(abs(deg - p), 180 - abs(deg - p)) > 4.0 for p in peaks):
            peaks.append(deg)
        if len(peaks) >= top:
            break
    return peaks


def score(yaml_path: Path) -> dict:
    arr, res = load_map(yaml_path)
    occ = arr < 80
    free = arr >= 230
    cell = res * res

    # 可放目标：腐蚀 0.30 m 后最大连通块
    from scipy.ndimage import binary_erosion, label
    k = max(1, int(round(0.30 / res)))
    er = binary_erosion(free, np.ones((2 * k + 1, 2 * k + 1)))
    lab, n = label(er)
    usable = 0
    if n:
        usable = int(max((lab == i).sum() for i in range(1, n + 1)))

    # 自由区最大连通块（判断有没有被切成碎片）
    labf, nf = label(free)
    big_free = 0
    if nf:
        big_free = int(max((labf == i).sum() for i in range(1, nf + 1)))

    # 墙体连续性：每个墙格 8 邻域里有多少墙格（>=3 视为"实心"，虚线段比例高）
    from scipy.ndimage import convolve
    nb = convolve(occ.astype(np.uint8), np.ones((3, 3), np.uint8), mode='constant') - occ
    continuous = float((nb[occ] >= 3).mean()) * 100.0 if occ.any() else 0.0

    dirs = dominant_dirs(occ)
    # 墙是正交的，直接在 0~180 上取 max-min 会得到 ~90（无意义）。
    # 先把方向折到 0~90，再按"最大间隙"切开做圆周统计，得到真正的簇内分散度。
    spread = 0.0
    if len(dirs) >= 2:
        wrapped = sorted(d % 90.0 for d in dirs)
        gaps = [(wrapped[(i + 1) % len(wrapped)] - wrapped[i]) % 90.0
                for i in range(len(wrapped))]
        cut = int(np.argmax(gaps))
        seq = wrapped[cut + 1:] + [w + 90.0 for w in wrapped[:cut + 1]]
        spread = seq[-1] - seq[0]
    pairs = 0
    for i in range(len(dirs)):
        for j in range(i + 1, len(dirs)):
            d = abs(((dirs[i] - dirs[j] + 90) % 180) - 90)
            if d < 5.0:
                pairs += 1

    return {
        'file': yaml_path.name,
        'size_m': (round(arr.shape[1] * res, 1), round(arr.shape[0] * res, 1)),
        'walls': int(occ.sum()),
        'free_m2': round(float(free.sum()) * cell, 1),
        'usable_m2': round(usable * cell, 1),
        'free_fragments': int(nf),
        'big_free_ratio': round(100.0 * big_free / max(1, int(free.sum())), 1),
        'continuous_ratio': round(continuous, 1),
        'dirs': [round(d, 1) for d in dirs],
        'spread_deg': round(spread, 1),
        'orthogonal_pairs': pairs,
    }


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    rows = []
    for p in sys.argv[1:]:
        try:
            rows.append(score(Path(p)))
        except Exception as exc:                      # noqa: BLE001
            print(f'{p}: 读取失败 {exc}')
    if not rows:
        return 2
    keys = ['file', 'size_m', 'walls', 'free_m2', 'usable_m2', 'free_fragments',
            'big_free_ratio', 'continuous_ratio', 'dirs', 'spread_deg',
            'orthogonal_pairs']
    width = max(len(r['file']) for r in rows)
    print(f'{"文件".ljust(width)}  {"尺寸m":>12} {"墙":>6} {"实心%":>6} {"空闲m2":>7} '
          f'{"可用m2":>7} {"主块%":>6} {"分散°":>6} {"正交":>4} {"方向":>22}')
    for r in rows:
        print(f'{r["file"].ljust(width)}  {str(r["size_m"]):>12} {r["walls"]:6d} '
              f'{r["continuous_ratio"]:6.1f} {r["free_m2"]:7.1f} {r["usable_m2"]:7.1f} '
              f'{r["big_free_ratio"]:6.1f} {r["spread_deg"]:6.1f} '
              f'{r["orthogonal_pairs"]:4d} {str(r["dirs"]):>22}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
