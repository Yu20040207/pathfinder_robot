"""Rigidly register one occupancy map onto another and stitch them.

merge_maps_kinematic can fall back to an identity transform, which pastes the two
maps side by side. This tool searches (dx, dy, dyaw) that minimises the distance
from the added map's obstacles to the base map's obstacles, reports the score so
the alignment is verifiable, and only then stitches the two grids.
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


def load(path):
    meta = yaml.safe_load(open(path, encoding='utf-8'))
    img = os.path.join(os.path.dirname(path), meta['image'])
    grid = np.array(Image.open(img))
    return grid, float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])


def obstacle_points(grid, res, ox, oy, step=2):
    ys, xs = np.nonzero(grid < 65)
    ys, xs = ys[::step], xs[::step]
    # 图像 y 轴向下，地图 y 轴向上
    wx = ox + (xs + 0.5) * res
    wy = oy + (grid.shape[0] - 1 - ys + 0.5) * res
    return np.stack([wx, wy], axis=1)


def score_transform(pts, dist, res, ox, oy, shape, dx, dy, dth, known=None,
                    min_points=200, scale=1.0):
    c, s = math.cos(dth), math.sin(dth)
    x = scale * (c * pts[:, 0] - s * pts[:, 1]) + dx
    y = scale * (s * pts[:, 0] + c * pts[:, 1]) + dy
    ix = ((x - ox) / res).astype(int)
    iy = (shape[0] - 1 - (y - oy) / res).astype(int)
    ok = (ix >= 0) & (ix < shape[1]) & (iy >= 0) & (iy < shape[0])
    if known is not None:
        # 只统计落在 base 已知区域（非未知）的点，新区域本身没有对应，不能计入打分。
        ok &= known[iy.clip(0, shape[0] - 1), ix.clip(0, shape[1] - 1)]
    if ok.sum() < min_points:
        return 9.9
    d = dist[iy[ok], ix[ok]]
    return float(np.percentile(d, 80))


def _search(args, pts, dist, bres, box, boy, shape, known):
    best = (9.9, 0.0, 0.0, 0.0)
    xs = np.arange(-25.0, 25.0, args.coarse_step)
    ys = np.arange(-25.0, 25.0, args.coarse_step)
    yaws = np.radians(np.arange(-180.0, 180.0, args.coarse_yaw))
    for dyaw in yaws:
        for dx in xs:
            for dy in ys:
                v = score_transform(pts, dist, bres, box, boy, shape, dx, dy, dyaw, known)
                if v < best[0]:
                    best = (v, dx, dy, dyaw)
    c0 = best
    for dyaw in c0[3] + np.radians(np.arange(-10, 10.1, 1.0)):
        for dx in np.arange(c0[1] - 0.4, c0[1] + 0.41, 0.05):
            for dy in np.arange(c0[2] - 0.4, c0[2] + 0.41, 0.05):
                v = score_transform(pts, dist, bres, box, boy, shape, dx, dy, dyaw, known)
                if v < best[0]:
                    best = (v, dx, dy, dyaw)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', required=True, help='map that defines the frame')
    ap.add_argument('--add', required=True, help='map to register onto base')
    ap.add_argument('--out', required=True)
    ap.add_argument('--coarse-step', type=float, default=0.4)
    ap.add_argument('--coarse-yaw', type=float, default=10.0)
    ap.add_argument('--max-score', type=float, default=0.10)
    ap.add_argument('--transform', default='',
                    help='fixed dx,dy,dyaw_deg from FFT registration; skips the search')
    args = ap.parse_args()

    bg, bres, box, boy = load(args.base)
    ag, ares, aox, aoy = load(args.add)
    dist = distance_transform_edt(~(bg < 65)) * bres
    known = bg != 205          # base 图的已知区域（障碍或空闲）
    pts = obstacle_points(ag, ares, aox, aoy)
    print(f'base obstacles {int((bg<65).sum())}, add sampled {len(pts)}')
    known = bg != 205

    if args.transform:
        parts = [float(v) for v in args.transform.split(',')]
        dx, dy, dyaw_deg = parts[0], parts[1], parts[2]
        tscale = parts[3] if len(parts) > 3 else 1.0
        dyaw = math.radians(dyaw_deg)
        v = score_transform(pts, dist, bres, box, boy, bg.shape, dx, dy, dyaw, known,
                            scale=tscale)
        print(f'fixed transform score {v:.3f}')
        best = (v, dx, dy, dyaw)
        best_scale = tscale
    else:
        best_scale = 1.0
        best = _search(args, pts, dist, bres, box, boy, bg.shape, known)
    print(f'search score {best[0]:.3f} at ({best[1]:+.2f},{best[2]:+.2f},{math.degrees(best[3]):+.1f}°)')
    if best[0] > args.max_score:
        print(f'ALIGN_FAIL score {best[0]:.3f} > {args.max_score}', file=sys.stderr)
        return 2

    # 世界范围
    _, dx, dy, dyaw = best
    c, s = math.cos(dyaw), math.sin(dyaw)
    corners = np.array([[0, 0], [ag.shape[1] * ares, 0],
                        [0, ag.shape[0] * ares], [ag.shape[1] * ares, ag.shape[0] * ares]])
    cc = corners @ np.array([[c, -s], [s, c]]).T + np.array([dx, dy]) + np.array([aox, aoy])
    min_x = min(box, cc[:, 0].min())
    max_x = max(box + bg.shape[1] * bres, cc[:, 0].max())
    min_y = min(boy, cc[:, 1].min())
    max_y = max(boy + bg.shape[0] * bres, cc[:, 1].max())
    width = int(math.ceil((max_x - min_x) / bres))
    height = int(math.ceil((max_y - min_y) / bres))
    out = np.full((height, width), 205, dtype=np.uint8)

    def paste(grid, res, ox, oy, transform=None, sc_scale=1.0):
        ys, xs = np.nonzero(grid < 65)
        wx = ox + (xs + 0.5) * res
        wy = oy + (grid.shape[0] - 1 - ys + 0.5) * res
        if transform is not None:
            tdx, tdy, tdth = transform
            tc, ts = math.cos(tdth), math.sin(tdth)
            wx, wy = sc_scale * (tc * wx - ts * wy) + tdx, sc_scale * (ts * wx + tc * wy) + tdy
        ix = ((wx - min_x) / bres).astype(int)
        iy = (height - 1 - (wy - min_y) / bres).astype(int)
        ok = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        out[iy[ok], ix[ok]] = 0

    paste(bg, bres, box, boy)
    paste(ag, ares, aox, aoy, (dx, dy, dyaw), best_scale)

    # 自由空间：两张图都明确的区域填白（障碍已画）
    free = np.zeros_like(out, dtype=bool)
    def mark_free(grid, res, ox, oy, transform=None, sc_scale=1.0):
        ys, xs = np.nonzero((grid > 200))
        wx = ox + (xs + 0.5) * res
        wy = oy + (grid.shape[0] - 1 - ys + 0.5) * res
        if transform is not None:
            tdx, tdy, tdth = transform
            tc, ts = math.cos(tdth), math.sin(tdth)
            wx, wy = sc_scale * (tc * wx - ts * wy) + tdx, sc_scale * (ts * wx + tc * wy) + tdy
        ix = ((wx - min_x) / bres).astype(int)
        iy = (height - 1 - (wy - min_y) / bres).astype(int)
        ok = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        free[iy[ok], ix[ok]] = True
    mark_free(bg, bres, box, boy)
    mark_free(ag, ares, aox, aoy, (dx, dy, dyaw), best_scale)
    out[(out != 0) & free] = 254

    Image.fromarray(out).save(args.out + '.pgm')
    with open(args.out + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f"image: {os.path.basename(args.out)}.pgm\n"
                f"resolution: {bres}\norigin: [{min_x:.3f}, {min_y:.3f}, 0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f'STITCHED {width}x{height} origin ({min_x:.2f},{min_y:.2f}) '
          f'obstacles {int((out==0).sum())}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
