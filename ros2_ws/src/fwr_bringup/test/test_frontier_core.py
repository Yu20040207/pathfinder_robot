from dataclasses import dataclass
from types import SimpleNamespace as NS

from fwr_bringup.frontier_core import FrontierGoal, choose_frontier


def grid(rows, resolution=.1):
    return NS(info=NS(width=len(rows[0]), height=len(rows), resolution=resolution,
                      origin=NS(position=NS(x=0., y=0.))),
              data=[v for row in rows for v in row])


def robot(x, y):
    return NS(pose=NS(position=NS(x=x, y=y)))


def open_map():
    rows = [[100] * 30 for _ in range(30)]
    for y in range(3, 26):
        for x in range(3, 26):
            rows[y][x] = 0
    for y in range(12, 19):
        for x in range(23, 26):
            rows[y][x] = -1
    return grid(rows)


def test_frontier_on_known_free_side_and_blacklist():
    m = open_map()
    goal = choose_frontier(m, robot(1.0, 1.5), set(), None)
    assert isinstance(goal, FrontierGoal)
    assert m.data[goal.grid_index] == 0
    assert goal.x < 2.3
    assert choose_frontier(m, robot(1.0, 1.5), {goal.grid_index}, None) != goal


def test_footprint_rejects_narrow_corridor_and_unreachable_region():
    rows = [[100] * 30 for _ in range(30)]
    for y in range(10, 14):
        for x in range(3, 27):
            rows[y][x] = 0
    rows[11][25] = -1
    assert choose_frontier(grid(rows), robot(.5, 1.15), set(), None) is None
    m = open_map()
    for y in range(3, 26):
        m.data[y * 30 + 15] = 100
    assert choose_frontier(m, robot(.8, 1.5), set(), None) is None


def test_keeps_current_goal_when_new_candidate_not_twenty_percent_better():
    m = open_map()
    first = choose_frontier(m, robot(1., 1.5), set(), None)
    assert choose_frontier(m, robot(1.01, 1.5), set(), first).grid_index == first.grid_index
