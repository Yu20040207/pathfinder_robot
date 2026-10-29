"""Compute the robot pose in a reference map and write slam_toolbox start pose.

Seeding logic matches the navigation one: checkpoint local refinement first,
then a global search. Used by tools/start_mapping_seeded.sh.
"""
import argparse
import math
import os
import re
import sys
import time

import numpy as np
import rclpy
import yaml
from PIL import Image
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan


def load_map(yaml_path):
    meta = yaml.safe_load(open(yaml_path, encoding='utf-8'))
    image = os.path.join(os.path.dirname(yaml_path), meta['image'])
    grid = np.array(Image.open(image))
    return meta, grid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--reference', required=True, help='reference map yaml')
    parser.add_argument('--config', required=True, help='extend config to update')
    parser.add_argument('--output', required=True, help='runtime config path')
    parser.add_argument('--checkpoint', default=os.path.expanduser('~/.config/fwr/last_pose.yaml'))
    parser.add_argument('--max-score', type=float, default=0.05)
    parser.add_argument('--prior-score', type=float, default=0.06)
    args = parser.parse_args()

    meta, grid = load_map(args.reference)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    occ = grid < 65
    dist = distance_transform_edt(~occ) * res
    height, width = occ.shape

    rclpy.init()
    node = Node('seed_mapping_pose')
    scans = []
    node.create_subscription(LaserScan, '/scan_mapping', lambda m: scans.append(m),
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    deadline = time.time() + 20.0
    while time.time() < deadline and len(scans) < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not scans:
        print('SEED_FAIL no scan', file=sys.stderr)
        return 2
    scan = scans[-1]
    pts = np.array([(r * math.cos(scan.angle_min + i * scan.angle_increment),
                     r * math.sin(scan.angle_min + i * scan.angle_increment))
                    for i, r in enumerate(scan.ranges)
                    if math.isfinite(r) and 0.1 < r < 8.0])

    def score(tx, ty, th):
        c, s = math.cos(th), math.sin(th)
        x = c * pts[:, 0] - s * pts[:, 1] + tx
        y = s * pts[:, 0] + c * pts[:, 1] + ty
        ix = ((x - ox) / res).astype(int)
        iy = (height - 1 - (y - oy) / res).astype(int)
        ok = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        if ok.sum() < len(pts) * 0.5:
            return 9.9
        return float(dist[iy[ok], ix[ok]].mean())

    best = (9.9, 0.0, 0.0, 0.0)
    # 1) checkpoint local refinement
    try:
        ck = yaml.safe_load(open(args.checkpoint, encoding='utf-8'))
        lx = ck['x'] + 0.21 * math.cos(ck['yaw'])
        ly = ck['y'] + 0.21 * math.sin(ck['yaw'])
        lyaw = ck['yaw'] + math.pi
        for dx in np.arange(-0.5, 0.51, 0.04):
            for dy in np.arange(-0.5, 0.51, 0.04):
                for dth in np.radians(np.arange(-15, 15.1, 1.5)):
                    v = score(lx + dx, ly + dy, lyaw + dth)
                    if v < best[0]:
                        best = (v, lx + dx, ly + dy, lyaw + dth)
        if best[0] <= args.prior_score:
            print(f'SEED_SOURCE checkpoint score={best[0]:.3f}')
    except (OSError, KeyError, TypeError, ValueError):
        pass
    # 2) global fallback
    if best[0] > args.prior_score:
        print('SEED_SOURCE global')
        for tx in np.arange(ox, ox + width * res, 0.25):
            for ty in np.arange(oy, oy + height * res, 0.25):
                for th in np.radians(np.arange(-180, 180, 8)):
                    v = score(tx, ty, th)
                    if v < best[0]:
                        best = (v, tx, ty, th)
        c0 = best
        for tx in np.arange(c0[1] - 0.25, c0[1] + 0.26, 0.04):
            for ty in np.arange(c0[2] - 0.25, c0[2] + 0.26, 0.04):
                for th in c0[3] + np.radians(np.arange(-8, 8.1, 1.0)):
                    v = score(tx, ty, th)
                    if v < best[0]:
                        best = (v, tx, ty, th)

    rclpy.shutdown()
    if best[0] > args.max_score:
        print(f'SEED_FAIL score={best[0]:.3f} > {args.max_score}', file=sys.stderr)
        return 3
    score_v, lx, ly, lyaw = best
    byaw = lyaw - math.pi
    bx = lx - 0.21 * math.cos(byaw)
    by = ly - 0.21 * math.sin(byaw)
    text = open(args.config, encoding='utf-8').read()
    text = re.sub(r'    map_start_pose: .*',
                  f'    map_start_pose: [{bx:.3f}, {by:.3f}, {byaw:.3f}]', text)
    open(args.output, 'w', encoding='utf-8').write(text)
    print(f'SEED base=({bx:.3f},{by:.3f},{byaw:.3f}) score={score_v:.3f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
