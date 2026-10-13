from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/fwr_vision"))
from fwr_vision.depth import sample_distance


def test_depth_median_in_meters_only_when_aligned():
    depth = np.full((10, 10), 1500, dtype=np.uint16)
    depth[5, 5] = 0
    assert sample_distance(depth, (2, 2, 6, 6), "16UC1", True) == 1.5
    assert sample_distance(depth, (2, 2, 6, 6), "16UC1", False) is None
    assert sample_distance(depth, (2, 2, 6, 6), "rgb8", True) is None
