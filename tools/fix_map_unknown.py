#!/usr/bin/env python3
"""把 Map Edit 面板保存的地图"修正"回原始约定。

为什么需要（2026-11-05 实测）：
    RViz 的 Map Edit 面板保存时，会把 OccupancyGrid 里所有 **未知(-1)** 像素
    写成 **自由(254)**。例如原图 未知 287286 像素，保存后变成 0 ——
    整个未探索区域都被当成了可走空间。

做法：
    以「编辑前的地图」为基准，只把用户真正的编辑应用上去：
      · 新图是墙(黑) 而 基准不是墙  → 画墙
      · 新图是白   而 基准是墙     → 擦墙
      其余像素（包括未知区）保持基准原样。

用法：
    python3 tools/fix_map_unknown.py <基准图.yaml> <新保存的.yaml> <输出.yaml>
"""
from __future__ import annotations

import sys

import numpy as np
import yaml
from PIL import Image


def load(path: str) -> tuple[np.ndarray, dict, str]:
    meta = yaml.safe_load(open(path, encoding='utf-8'))
    img_path = path.rsplit('/', 1)[0] + '/' + meta['image']
    return np.array(Image.open(img_path)), meta, img_path


def apply_edits(base: np.ndarray, edited: np.ndarray) -> tuple[np.ndarray, int, int]:
    """把 edited 的编辑应用到 base 上，保留 base 的未知区。返回 (图, 画墙数, 擦墙数)。"""
    if base.shape != edited.shape:
        raise ValueError('尺寸不一致: %s vs %s' % (base.shape, edited.shape))
    out = base.copy()
    drew = (edited < 65) & (base >= 65)          # 新增的墙
    erased = (edited > 250) & (base < 65)        # 被擦掉的墙
    out[drew] = 0
    out[erased] = 254
    return out, int(drew.sum()), int(erased.sum())


def main(argv: list[str]) -> int:
    if len(argv) != 4:
        print(__doc__)
        return 2
    base_path, edited_path, out_path = argv[1:4]
    base, base_meta, _ = load(base_path)
    edited, _, _ = load(edited_path)
    out, n_draw, n_erase = apply_edits(base, edited)
    out_base = out_path[:-5] if out_path.endswith('.yaml') else out_path
    Image.fromarray(out).save(out_base + '.pgm')
    meta = dict(base_meta)
    meta['image'] = out_base.rsplit('/', 1)[-1] + '.pgm'
    with open(out_base + '.yaml', 'w', encoding='utf-8') as handle:
        yaml.safe_dump(meta, handle, default_flow_style=None, sort_keys=False)
    print('修正完成: %s.pgm/.yaml  画墙 %d 像素, 擦墙 %d 像素（未知区已保留）' % (
        out_base, n_draw, n_erase))
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
