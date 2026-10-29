from fwr_bringup.fixed_goal_core import map_available, accept_map_message


def test_live_slam_map_requires_fresh_stamp_and_receipt():
    assert accept_map_message(False, True, 0.2)
    assert not accept_map_message(False, True, 5.)
    assert map_available(False, True, 9.5, 10.)
    assert not map_available(False, True, 8., 10.)


def test_saved_map_keeps_cached_map_available():
    assert accept_map_message(True, True, 900.)
    # Nav2 map_server publishes a saved map with a zero header stamp on Humble.
    assert accept_map_message(True, False, 900.)
    assert map_available(True, True, 1., 100.)
    assert not map_available(True, False, 1., 100.)
