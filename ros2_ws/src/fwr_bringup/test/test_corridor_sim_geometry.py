"""Keep the Gazebo drive geometry aligned with the measured 65 mm wheels."""

from pathlib import Path
from xml.etree import ElementTree


WORLD = Path(__file__).resolve().parents[1] / 'worlds' / 'corridor_c1.sdf'


def test_drive_geometry_matches_chassis_measurements():
    model = ElementTree.parse(WORLD).getroot().find('.//model[@name="fwr_chassis"]')
    drive = model.find('./plugin[@name="gz::sim::systems::DiffDrive"]')
    assert float(drive.findtext('wheel_separation')) == .263
    assert float(drive.findtext('wheel_radius')) == .0325
    for name, expected_y in [('left_drive_wheel', .1315),
                             ('right_drive_wheel', -.1315)]:
        wheel = model.find(f'./link[@name="{name}"]')
        x, y, z, *_ = map(float, wheel.findtext('pose').split())
        assert x == -.13
        assert y == expected_y
        assert z == .0325
        assert float(wheel.findtext('./collision/geometry/cylinder/radius')) == .0325
