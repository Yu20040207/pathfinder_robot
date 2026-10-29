import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
from check_fixed_goal_reroute_sim import chassis_obstacle_clearance


def test_box_circle_clearance_accounts_for_heading():
    assert math.isclose(chassis_obstacle_clearance((0., 0., 0.), (1., 0.)), .56)
    assert math.isclose(chassis_obstacle_clearance((0., 0., 0.), (0., .5)), .145)
    assert math.isclose(chassis_obstacle_clearance((0., 0., math.pi / 2), (0., 1.)), .56)
    assert chassis_obstacle_clearance((0., 0., 0.), (.3, 0.)) < 0
