"""Depth extraction only for verified, pixel aligned images."""

import numpy as np


def sample_distance(depth: np.ndarray, box: tuple[int, int, int, int], encoding: str, aligned: bool) -> float | None:
    if not aligned or depth is None or encoding != "16UC1" or depth.ndim != 2:
        return None
    x, y, w, h = box
    x0 = max(0, x + w // 4)
    x1 = min(depth.shape[1], x + 3 * w // 4)
    y0 = max(0, y + h // 4)
    y1 = min(depth.shape[0], y + 3 * h // 4)
    if x1 <= x0 or y1 <= y0:
        return None
    pixels = depth[y0:y1, x0:x1]
    valid = pixels[(pixels > 0) & (pixels < 10000)]
    return float(np.median(valid)) / 1000.0 if valid.size else None
