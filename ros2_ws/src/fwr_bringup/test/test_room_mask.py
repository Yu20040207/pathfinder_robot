"""Adaptive room segmentation: size-independent, door-width based."""
import numpy as np

from fwr_bringup.room_mask import segment_room


def _two_rooms(h=24, w=60, wall_x=30, door_y=(11, 12), res=0.05):
    free = np.zeros((h, w), dtype=bool)
    free[:, :wall_x] = True
    free[:, wall_x + 1:] = True
    free[:, wall_x] = False
    # Door gap through the dividing wall.
    free[door_y[0]:door_y[1] + 1, wall_x] = True
    return free, res


def test_room_mask_keeps_current_room_and_excludes_other_room():
    free, res = _two_rooms()
    # Door is 2 cells (0.10 m); clearance 0.20 m cuts it.
    room = segment_room(free, (12, 10), res, door_clearance_m=0.20)
    assert room is not None
    assert room[12, 10], 'robot must be inside its room mask'
    assert room[3, 3], 'room must cover free space, independent of size'
    assert not room[12, 45], 'adjacent room across a door must be excluded'


def test_room_mask_supports_different_room_sizes():
    free, res = _two_rooms(h=24, w=60, wall_x=30)
    # Make the right room much larger (wider); left remains narrow.
    big = np.zeros((40, 80), dtype=bool)
    big[:, :30] = True
    big[:, 31:] = True
    big[:, 30] = False
    big[18:20, 30] = True
    room = segment_room(big, (20, 20), res, door_clearance_m=0.20)
    assert room is not None
    assert room[5, 5] and not room[20, 60]


def test_room_mask_falls_back_when_map_too_small():
    free = np.zeros((5, 5), dtype=bool)
    free[1:4, 1:4] = True
    # Clearance erosion removes every cell; no fabricated room.
    assert segment_room(free, (2, 2), 0.05, door_clearance_m=0.50) is None
