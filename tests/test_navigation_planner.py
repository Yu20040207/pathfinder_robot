import math
from pathlib import Path
import sys

import pytest

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "ros2_ws/src/fwr_navigation"))

from fwr_navigation.planner import GridMap, PlanningError, plan_path
from fwr_navigation.node import grid_from_message, path_message
from nav_msgs.msg import OccupancyGrid


def grid(rows, *, resolution=1.0, origin=(0.0, 0.0, 0.0)):
    return GridMap(len(rows[0]), len(rows), resolution, *origin,
                   tuple(value for row in rows for value in row))


def test_direct_path_and_rotated_map_coordinates():
    board = grid([[0] * 4 for _ in range(4)], origin=(10.0, 20.0, math.pi / 2))
    start = board.cell_center(0, 0)
    goal = board.cell_center(3, 3)
    path = plan_path(board, start, goal, clearance_m=0)
    assert path[0] == (0, 0)
    assert path[-1] == (3, 3)
    assert board.world_to_cell(*goal) == (3, 3)


def test_obstacle_detour_does_not_cut_diagonal_corner():
    board = grid([
        [0, 100, 0, 0],
        [0, 100, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ])
    path = plan_path(board, board.cell_center(0, 0), board.cell_center(3, 0), clearance_m=0)
    assert all(board.value(x, y) == 0 for x, y in path)
    assert (1, 2) in path
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        if ax != bx and ay != by:
            assert board.value(ax, by) == 0
            assert board.value(bx, ay) == 0


def test_unknown_and_clearance_block_route():
    board = grid([[0, 0, 0], [0, -1, 0], [0, 0, 0]])
    with pytest.raises(PlanningError, match="blocked"):
        plan_path(board, board.cell_center(0, 0), board.cell_center(2, 2), clearance_m=1.0)
    with pytest.raises(PlanningError, match="blocked"):
        plan_path(board, board.cell_center(1, 1), board.cell_center(2, 2), clearance_m=0)
    wall = grid([[0, 100, 0], [0, 100, 0], [0, 100, 0]])
    with pytest.raises(PlanningError, match="no path"):
        plan_path(wall, wall.cell_center(0, 1), wall.cell_center(2, 1), clearance_m=0)


def test_rejects_invalid_grid_and_out_of_bounds_goal():
    with pytest.raises(ValueError):
        grid([[0, 0], [0]])
    board = grid([[0, 0], [0, 0]])
    with pytest.raises(PlanningError, match="outside"):
        plan_path(board, board.cell_center(0, 0), (4, 4), clearance_m=0)


def test_ros_map_and_path_conversion():
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = 2
    message.info.height = 2
    message.info.resolution = 0.5
    message.info.origin.orientation.w = 1.0
    message.data = [0, 0, 0, 0]
    board = grid_from_message(message)
    path = path_message(board, [(0, 0), (1, 1)], message.header, (0.25, 0.25), (0.75, 0.75))
    assert path.header.frame_id == "map"
    assert len(path.poses) >= 2
    assert path.poses[0].pose.position.x == 0.25
    assert path.poses[-1].pose.position.x == 0.75


def test_same_cell_path_keeps_exact_goal():
    board = grid([[0, 0], [0, 0]])
    cells = plan_path(board, (0.1, 0.1), (0.9, 0.9), clearance_m=0)
    message = OccupancyGrid()
    message.header.frame_id = "map"
    path = path_message(board, cells, message.header, (0.1, 0.1), (0.9, 0.9))
    assert path.poses[-1].pose.position.x == 0.9


def test_clearance_includes_obstacle_cell_edges():
    board = grid([[0, 100, 0], [0, 0, 0], [0, 0, 0]])
    with pytest.raises(PlanningError, match="blocked"):
        plan_path(board, board.cell_center(0, 0), board.cell_center(2, 2), clearance_m=0.6)
