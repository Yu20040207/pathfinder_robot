from types import SimpleNamespace as N
import math

from fwr_bringup.fixed_goal_core import validate_fixed_goal, validate_two_point_goals


def grid():
    data = [0] * (40 * 40)
    info = N(width=40, height=40, resolution=.05,
             origin=N(position=N(x=-1., y=-1.), orientation=N(x=0., y=0., z=0., w=1.)))
    return N(info=info, data=data)


def large_grid():
    info = N(width=160, height=160, resolution=.05,
             origin=N(position=N(x=-4., y=-4.), orientation=N(x=0., y=0., z=0., w=1.)))
    return N(info=info, data=[0] * (160 * 160))


def test_accepts_short_known_free_goal():
    assert validate_fixed_goal(grid(), 0., 0., .35, 0., 0., .5)[0]


def test_rejects_invalid_distance_and_obstacles():
    g = grid()
    for x, y, yaw in ((math.nan, 0., 0.), (0., 0., math.inf), (2., 0., 0.), (.6, 0., 0.)):
        assert not validate_fixed_goal(g, 0., 0., x, y, yaw, .5)[0]
    idx = 20 * 40 + 27
    g.data[idx] = 100
    assert not validate_fixed_goal(g, 0., 0., .35, 0., 0., .5)[0]
    g.data[idx] = -1
    assert not validate_fixed_goal(g, 0., 0., .35, 0., 0., .5)[0]


def test_rejects_goal_without_footprint_clearance():
    # 目标点要求净空 >= 0.30 m；这里放一个 0.25 m 内的障碍，必须拒绝。
    g = grid()
    g.data[20 * 40 + 32] = 100
    assert not validate_fixed_goal(g, 0., 0., .35, 0., 0., .5)[0]


def test_two_point_checks_both_legs_and_second_goal_footprint():
    g = grid()
    assert validate_two_point_goals(g, 0., 0., (.1, 0., 0.), (.4, 0., 0.), .5)[0]
    ok, reason = validate_two_point_goals(g, 0., 0., (.1, 0., 0.), (.9, 0., 0.), .5)
    assert not ok and 'second' in reason
    # 未知格放在 0.25 m 处（新阈值 0.30 m 内），第二段必须拒绝。
    g.data[20 * 40 + 32] = -1
    ok, reason = validate_two_point_goals(g, 0., 0., (.1, 0., 0.), (.4, 0., 0.), .5)
    assert not ok and 'second' in reason


def test_second_leg_keeps_goal_tolerance_inside_trial_limit():
    g = grid()
    ok, reason = validate_two_point_goals(g, 0., 0., (.2, 0., 0.), (.51, 0., 0.), .5)
    assert not ok and 'second' in reason
    assert validate_two_point_goals(g, 0., 0., (.2, 0., 0.), (.5, 0., 0.), .5)[0]


def test_zero_distance_limit_accepts_long_known_free_two_point_route():
    g = large_grid()
    assert validate_two_point_goals(g, 0., 0., (1.0, 0., 0.), (2.5, 0., 0.), 0.)[0]
    assert validate_fixed_goal(g, 0., 0., 2.5, 0., 0., 0.)[0]


def test_unlimited_route_still_rejects_unknown_goal_footprint():
    g = large_grid()
    g.data[80 * 160 + 130] = -1
    ok, reason = validate_two_point_goals(g, 0., 0., (1.0, 0., 0.), (2.5, 0., 0.), 0.)
    assert not ok and 'second' in reason
