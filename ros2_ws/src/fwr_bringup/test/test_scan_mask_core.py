import math
import struct
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fwr_bringup.scan_mask_core import mask_rear_ranges


def test_rear_is_masked_and_front_side_preserved():
    values = [1.0] * 9
    out = mask_rear_ranges(values, -math.pi, math.pi / 4, math.pi / 2)
    assert [math.isnan(v) for v in out] == [False, False, True, True, True, True, True, False, False]
    assert values == [1.0] * 9


def test_negative_increment_and_wrap():
    out = mask_rear_ranges([1.0] * 5, 3 * math.pi / 2, -math.pi / 4, math.pi / 2)
    assert [math.isnan(v) for v in out] == [True, False, False, False, True]


def test_float32_laserscan_boundary_is_included():
    f32 = lambda value: struct.unpack('f', struct.pack('f', value))[0]
    out = mask_rear_ranges([1.0] * 9, f32(-math.pi), f32(math.pi / 4), math.pi / 2)
    assert math.isnan(out[2]) and math.isnan(out[6])


def test_preserves_unmasked_special_values():
    out = mask_rear_ranges([float('inf'), float('nan'), 3.0], math.pi, -math.pi, 0.7)
    assert math.isinf(out[0]) and math.isnan(out[1]) and out[2] == 3.0


@pytest.mark.parametrize('angle_min,increment,half', [
    (float('nan'), 1.0, 1.0), (0.0, 0.0, 1.0), (0.0, float('inf'), 1.0),
    (0.0, 1.0, 0.0), (0.0, 1.0, math.pi),
])
def test_invalid_geometry_rejected(angle_min, increment, half):
    with pytest.raises(ValueError):
        mask_rear_ranges([1.0], angle_min, increment, half)
