"""Adaptive room mask from a live occupancy grid.

The room is not a fixed radius: erode known free space by half a typical
door width, take the connected core containing the robot, then dilate that
core back inside free space.  Adjacent rooms/corridors connected only by a
door are separated by the erosion, so target filtering follows the actual
room regardless of its size.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage

_NEIGHBOURS = np.ones((3, 3), dtype=bool)


def segment_room(free: np.ndarray, robot: tuple[int, int] | None,
                 resolution: float, door_clearance_m: float = 0.50
                 ) -> np.ndarray | None:
    """Return a boolean room mask, or None when no usable core exists.

    ``free`` is the known-free mask (row, col).  Unknown and occupied cells
    are not part of the room.  Doorways narrower than ``2 * clearance`` are
    cut by the erosion; wider openings intentionally remain one region, since
    two rooms truly open to each other have no physical door to stop at.
    """
    if free is None or free.ndim != 2 or not free.any():
        return None
    if robot is None:
        return None
    h, w = free.shape
    r, c = int(robot[0]), int(robot[1])
    if not (0 <= r < h and 0 <= c < w):
        return None
    if resolution <= 0:
        return None
    cells = max(1, int(round(float(door_clearance_m) / float(resolution))))
    clearance = ndimage.distance_transform_edt(free)
    core = free & (clearance >= cells)
    labels, count = ndimage.label(core, structure=_NEIGHBOURS)
    if count == 0:
        return None
    if core[r, c]:
        cid = int(labels[r, c])
    else:
        # Start can be within the door/wall clearance band.  Attach it to the
        # nearest core cell rather than silently disabling the room mask.
        ys, xs = np.nonzero(core)
        k = int(np.argmin((ys - r) ** 2 + (xs - c) ** 2))
        cid = int(labels[ys[k], xs[k]])
    if cid <= 0:
        return None
    comp = labels == cid
    if not comp.any():
        return None
    room = ndimage.binary_dilation(comp, iterations=cells) & free
    return room
