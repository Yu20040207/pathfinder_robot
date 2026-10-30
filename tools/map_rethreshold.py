#!/usr/bin/env python3
"""把「带概率梯度」的 Cartographer PGM 重新二值化成标准导航地图。

背景：cartographer_pbstream_to_ros_map 输出的 PGM 保留了每个格子的占用概率
（像素值 = 255×(1-p)），但 nav2 map_server 只吃三值图（0/205/254）。
直接用这个文件当导航图，墙会被当成"未知"，看起来断断续续。

用法：
  .venv/bin/python tools/map_rethreshold.py 输入.pgm 输出名 \
      --occupied 128 --free 240 --yaml-origin-x -13.8 --yaml-origin-y -11.4

  --occupied  像素值 <= 该值 → 判为墙（越小越保守）
  --free      像素值 >= 该值 → 判为空闲
  中间值 → 未知

坐标修正：本次源 PGM 的 origin 与 yaml 不一致时用 --yaml-origin-x/y 覆写。
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('pgm', help='输入概率图（cartographer 输出）')
    ap.add_argument('out_stem', help='输出文件名（不含扩展名）')
    ap.add_argument('--occupied', type=int, default=128,
                    help='像素 <= 该值判为占用（默认 128）')
    ap.add_argument('--free', type=int, default=240,
                    help='像素 >= 该值判为空闲（默认 240）')
    ap.add_argument('--weak-occupied', type=int, default=0,
                    help='像素 129~该值 视为弱证据墙（0=不启用，越大补得越多）')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--origin-x', type=float, default=None)
    ap.add_argument('--origin-y', type=float, default=None)
    args = ap.parse_args()

    src = np.array(Image.open(args.pgm).convert('L')).astype(int)
    h, w = src.shape
    # Cartographer 的编码：像素 = 255×(1-p_occ)
    #   0   = 100% 墙       127 = 50% 墙
    #   128 = 未知（恰好等于 50%，与"半信半疑的墙"同值）
    #   129~254 = 概率从 50% 递减到 0（弱证据 / 渐变）
    #   255 = 空闲
    out = np.full((h, w), 205, dtype=np.uint8)       # 默认未知
    out[src >= args.free] = 254                       # 空闲
    out[src <= args.occupied] = 0                     # 确定的墙（<=127）
    if args.weak_occupied > 0:
        weak = (src > 128) & (src <= args.weak_occupied)
        out[weak] = 0                                 # 把弱证据也算成墙
    Image.fromarray(out).save(f'{args.out_stem}.pgm')

    meta = {
        'image': Path(f'{args.out_stem}.pgm').name,
        'mode': 'trinary',
        'resolution': args.resolution,
        'origin': [args.origin_x if args.origin_x is not None else 0.0,
                   args.origin_y if args.origin_y is not None else 0.0,
                   0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(f'{args.out_stem}.yaml', 'w') as fh:
        yaml.safe_dump(meta, fh, default_flow_style=None, sort_keys=False)

    occ = int((out == 0).sum())
    free = int((out == 254).sum())
    unk = int((out == 205).sum())
    print(f'{args.out_stem}: {w}x{h}  墙 {occ}  空闲 {free} ({free*args.resolution**2:.1f} m²)  '
          f'未知 {unk}')
    print(f'  阈值: 占用 <= {args.occupied} / 空闲 >= {args.free}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
