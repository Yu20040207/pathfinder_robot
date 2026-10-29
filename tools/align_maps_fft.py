"""Register two occupancy maps with FFT cross-correlation over a yaw sweep.

Point-sampling registration misses thin wall alignments; cross-correlation in the
frequency domain evaluates every translation at once, which is far more robust for
occupancy grids. The result is reported as a normalised peak score so the fit can
be judged before stitching.
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import rotate, distance_transform_edt


def load(path):
    meta = yaml.safe_load(open(path, encoding='utf-8'))
    img = os.path.join(os.path.dirname(path), meta['image'])
    grid = np.array(Image.open(img))
    return grid, float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base', required=True)
    ap.add_argument('--add', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--yaw-step', type=float, default=2.0)
    ap.add_argument('--yaw-range', type=float, default=180.0)
    ap.add_argument('--min-score', type=float, default=0.25)
    args = ap.parse_args()

    bg, bres, box, boy = load(args.base)
    ag, ares, aox, aoy = load(args.add)
    if abs(bres - ares) > 1e-6:
        print('FAIL resolution mismatch', file=sys.stderr)
        return 2
    b_occ = (bg < 65).astype(np.float32)
    a_occ = (ag < 65).astype(np.float32)
    b_dist = distance_transform_edt(~(bg < 65)) * bres

    pad = int(max(bg.shape[0], ag.shape[0]) * 0.5)
    B = np.fft.rfft2(b_occ, s=(bg.shape[0] + 2 * pad, bg.shape[1] + 2 * pad))
    n_b = b_occ.sum()

    best = (0.0, 0.0, 0.0, 0.0)
    for yaw_deg in np.arange(-args.yaw_range, args.yaw_range, args.yaw_step):
        rot = rotate(a_occ, yaw_deg, order=1, reshape=True) if abs(yaw_deg) > 1e-9 else a_occ
        A = np.fft.rfft2(rot, s=B.shape[-2:] if False else None) if False else None
        # numpy needs identical shapes: zero-pad rotated image to the base canvas
        H, W = B.shape[0], (B.shape[1] - 1) * 2
        canvas = np.zeros((H, W), dtype=np.float32)
        h = min(H, rot.shape[0]); w = min(W, rot.shape[1])
        y0 = (H - h) // 2; x0 = (W - w) // 2
        canvas[y0:y0 + h, x0:x0 + w] = rot[:h, :w]
        A = np.fft.rfft2(canvas)
        corr = np.fft.irfft2(A * np.conj(B), s=(H, W))
        peak = float(corr.max())
        norm = peak / max(1.0, min(n_b, canvas.sum()))
        if norm > best[0]:
            iy, ix = np.unravel_index(np.argmax(corr), corr.shape)
            best = (norm, ix, iy, yaw_deg, canvas, y0, x0, rot.shape)
    norm, ix, iy, yaw_deg, canvas, y0, x0, rshape = best
    print(f'best normalized peak {norm:.3f} at yaw {yaw_deg:+.1f}° shift ({ix},{iy})')
    if norm < args.min_score:
        print(f'ALIGN_FAIL peak {norm:.3f} < {args.min_score}', file=sys.stderr)
        return 3

    # 把互相关得到的平移换算成世界坐标变换
    H, W = canvas.shape
    # 相关位移：图像 ix 对应 +x，iy 对应 y 轴翻转
    cam_x = ox_shift = (ix - W // 2)
    cam_y = (iy - H // 2)
    a_center = np.array(rshape) / 2.0
    # 新图中每格的世界尺寸一致，直接换算
    dx_world = (cam_x - (x0 - a_center[1])) * bres
    dy_world = -(cam_y - (y0 - a_center[0])) * bres
    print(f'translation ≈ ({dx_world:+.2f}, {dy_world:+.2f}) m, yaw {yaw_deg:+.1f}°')
    np.save(args.out + '_registration.npy',
            np.array([dx_world, dy_world, math.radians(yaw_deg)]))
    print('SAVED registration to', args.out + '_registration.npy')
    return 0


if __name__ == '__main__':
    sys.exit(main())
