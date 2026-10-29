from fwr_bringup.exploration_core import ExplorationStateMachine


def test_trial_timeout_only_stops_exploration_not_fixed_route():
    from fwr_bringup.exploration_core import trial_duration_expired

    assert trial_duration_expired(0., 121., 120., fixed_goal=False)
    assert not trial_duration_expired(0., 121., 120., fixed_goal=True)
    assert not trial_duration_expired(None, 121., 120., fixed_goal=False)


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


def test_map_to_odom_jump_detector():
    from fwr_bringup.exploration_core import map_frame_jumped
    assert not map_frame_jumped((1., 1.), (0., 0.), (1.03, 1.), (.03, 0.))
    assert map_frame_jumped((1., 1.), (0., 0.), (2.0, 1.), (.03, 0.))


def test_stop_waits_for_pending_send_and_terminal_cancel():
    sm = ExplorationStateMachine()
    sm.start()
    assert sm.update(True, True, 0, 'sending', 1).action == 'wait'
    sm.stop()
    assert sm.update(True, True, 0, 'sending', 2).action == 'cancel'
    assert sm.update(True, True, 0, 'canceling_pending', 3).action == 'wait'
    assert sm.update(True, True, 0, 'canceling', 4).action == 'wait'
    assert sm.update(True, True, 0, 'canceled', 5).action == 'stop'


def test_fixed_goal_dispatches_once_then_stops_on_success():
    sm = ExplorationStateMachine()
    sm.start_fixed()
    assert sm.update(True, True, 0, 'idle', 1).action == 'dispatch'
    assert sm.update(True, True, 0, 'sending', 2).state == 'navigating'
    assert sm.update(True, True, 0, 'active', 3).state == 'navigating'
    result = sm.update(True, True, 0, 'succeeded', 4)
    assert result.action == 'stop' and result.state == 'stopped'
    assert not sm.enabled
    assert sm.update(True, True, 0, 'succeeded', 5).action == 'stop'


def test_fixed_goal_failure_fault_and_stop_do_not_retry():
    sm = ExplorationStateMachine()
    sm.start_fixed()
    sm.update(True, True, 0, 'active', 1)
    assert sm.update(True, True, 0, 'failed', 2).action == 'stop'
    assert not sm.enabled
    sm.start_fixed()
    assert sm.update(True, True, 1, 'active', 3).action == 'cancel'
    assert sm.update(True, True, 1, 'canceled', 4).action == 'stop'
    assert not sm.enabled


def test_two_point_goal_dispatches_second_leg_only_after_first_succeeds():
    sm = ExplorationStateMachine()
    sm.start_fixed(legs=2)
    assert sm.update(True, True, 0, 'idle', 1).action == 'dispatch'
    assert sm.fixed_leg_index == 0
    assert sm.update(True, True, 0, 'active', 2).action == 'wait'
    assert sm.update(True, True, 0, 'succeeded', 3).action == 'dispatch'
    assert sm.fixed_leg_index == 1
    assert sm.update(True, True, 0, 'active', 4).action == 'wait'
    assert sm.update(True, True, 0, 'succeeded', 5).action == 'stop'
    assert not sm.enabled


def test_two_point_goal_never_starts_second_leg_after_failure_or_stop():
    for terminal in ('failed', 'canceled'):
        sm = ExplorationStateMachine()
        sm.start_fixed(legs=2)
        sm.update(True, True, 0, 'idle', 1)
        assert sm.update(True, True, 0, terminal, 2).action == 'stop'
        assert sm.fixed_leg_index == 0
    sm = ExplorationStateMachine()
    sm.start_fixed(legs=2)
    sm.update(True, True, 0, 'idle', 1)
    sm.stop()
    assert sm.update(True, True, 0, 'active', 2).action == 'cancel'
    assert sm.update(True, True, 0, 'canceled', 3).action == 'stop'
    assert sm.fixed_leg_index == 0


def test_two_point_goal_fault_between_legs_prevents_second_dispatch():
    sm = ExplorationStateMachine()
    sm.start_fixed(legs=2)
    sm.update(True, True, 0, 'idle', 1)
    assert sm.update(True, True, 7, 'succeeded', 2).action == 'stop'
    assert sm.fixed_leg_index == 0
