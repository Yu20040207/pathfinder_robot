import math
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "ros2_ws/src/fwr_navigation"))

from fwr_navigation.simulation import DifferentialPose, WaypointFollower
from fwr_navigation.planner import GridMap, plan_path


def test_differential_motion_straight_turn_arc():
    pose = DifferentialPose(0.0, 0.0, 0.0)
    assert pose.step(1.0, 1.0, 1.0, 0.5).x == 1.0
    turn = pose.step(-0.5, 0.5, 1.0, 0.5)
    assert math.isclose(turn.x, 0.0, abs_tol=1e-9)
    assert math.isclose(turn.yaw, 2.0, abs_tol=1e-9)
    arc = pose.step(0.0, 1.0, 1.0, 1.0)
    assert arc.x > 0 and arc.y > 0


def test_follower_limits_speed_and_stops_at_goal():
    follower = WaypointFollower(max_linear=0.4, max_angular=1.0)
    follower.set_path([(0.0, 0.0), (2.0, 0.0)])
    command = follower.command(DifferentialPose(0.0, 0.0, 0.0))
    assert 0 < command.linear <= 0.4
    assert command.angular == 0
    at_goal = follower.command(DifferentialPose(2.0, 0.0, 0.0))
    assert at_goal.linear == at_goal.angular == 0
    assert follower.reached


def test_follower_turns_before_driving_and_rejects_empty_path():
    follower = WaypointFollower(max_linear=0.3, max_angular=0.8)
    follower.set_path([(0.0, 0.0), (0.0, 2.0)])
    command = follower.command(DifferentialPose(0.0, 0.0, 0.0))
    assert command.linear == 0
    assert 0 < command.angular <= 0.8
    follower.set_path([])
    assert follower.command(DifferentialPose(0.0, 0.0, 0.0)).linear == 0


def test_demo_route_reaches_goal_without_entering_obstacle_cells():
    board = GridMap(12, 12, 0.25, 0, 0, 0,
                    tuple(100 if x in (5, 6) and y < 8 else 0
                          for y in range(12) for x in range(12)))
    start, goal = (0.625, 0.625), (2.375, 0.625)
    cells = plan_path(board, start, goal, clearance_m=0.05)
    follower = WaypointFollower(max_linear=0.35, max_angular=0.9,
                                position_tolerance=0.07)
    follower.set_path([start] + [board.cell_center(*cell) for cell in cells[1:-1]] + [goal])
    pose = DifferentialPose(*start, 0.0)
    for _ in range(1600):
        command = follower.command(pose)
        separation = 0.4
        pose = pose.step(command.linear - command.angular * separation / 2,
                         command.linear + command.angular * separation / 2,
                         0.05, separation)
        assert board.value(*board.world_to_cell(pose.x, pose.y)) == 0
        if follower.reached:
            break
    assert follower.reached
    assert math.hypot(pose.x - goal[0], pose.y - goal[1]) < 0.07
