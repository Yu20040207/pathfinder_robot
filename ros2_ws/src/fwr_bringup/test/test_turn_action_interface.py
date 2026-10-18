from fwr_interfaces.action import TurnRelative


def test_turn_action_has_explicit_modes_and_result():
    goal = TurnRelative.Goal()
    assert goal.PIVOT == 1 and goal.ARC == 2
    goal.angle_deg = -5.0
    goal.mode = goal.PIVOT
    result = TurnRelative.Result()
    result.success = False
    result.reason = 'imu_stale'
    result.actual_angle_deg = -1.0
    assert (goal.angle_deg, result.reason) == (-5.0, 'imu_stale')
