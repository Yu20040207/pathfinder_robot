from fwr_bringup.exploration_core import ExplorationStateMachine


def test_disabled_start_and_explicit_start_stop():
    sm = ExplorationStateMachine()
    assert sm.update(True, True, 0, 'idle', 0).action == 'stop'
    sm.start()
    assert sm.update(True, True, 0, 'idle', 1).action == 'dispatch'
    sm.stop()
    assert sm.update(True, True, 0, 'active', 2).action == 'cancel'
    assert sm.update(True, True, 0, 'canceled', 3).action == 'stop'
    assert ExplorationStateMachine().update(True, True, 0, 'idle', 4).action == 'stop'


def test_failure_cancels_before_recovery_and_stops_after_limit():
    sm = ExplorationStateMachine()
    sm.start()
    assert sm.update(True, True, 0, 'active', 1, goal_index=7).action == 'wait'
    assert sm.update(True, True, 0, 'failed', 2, goal_index=7).action == 'cancel'
    assert sm.update(True, True, 0, 'canceling', 3, goal_index=7).action == 'wait'
    assert sm.update(True, True, 0, 'canceled', 4, goal_index=7).action == 'recover'
    sm.recovery_finished(False)
    assert 7 not in sm.blocked
    assert sm.update(True, True, 0, 'failed', 5, goal_index=7).action == 'cancel'
    assert 7 in sm.blocked


def test_fault_or_pose_jump_stops_and_cannot_resume():
    sm = ExplorationStateMachine()
    sm.start()
    assert sm.update(True, True, 1, 'active', 0).action == 'cancel'
    assert not sm.enabled
    sm.start()
    assert sm.update(True, False, 0, 'idle', 1).action == 'stop'
    assert not sm.enabled
