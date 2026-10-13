from fwr_voice.task_slot import TaskSlot


def test_only_one_goal_can_be_pending():
    slot = TaskSlot()
    assert slot.reserve()
    assert not slot.reserve()
    assert slot.is_busy()


def test_stop_during_pending_cancels_when_goal_arrives():
    slot = TaskSlot()
    slot.reserve()
    assert slot.request_cancel() is None
    handle = object()
    assert slot.accept(handle)
    assert slot.request_cancel() is handle
    slot.finish()
    assert not slot.is_busy()


def test_new_motion_replaces_active_goal_after_cancel():
    slot = TaskSlot()
    assert slot.reserve()
    old = object()
    assert not slot.accept(old)
    start_now, cancel = slot.reserve_or_replace("right")
    assert not start_now and cancel is old
    assert slot.finish() == "right"
    assert slot.is_busy()
    assert not slot.accept(object())
    assert slot.finish() is None


def test_stop_discards_queued_replacement():
    slot = TaskSlot()
    slot.reserve()
    slot.reserve_or_replace("forward")
    slot.request_cancel()
    assert slot.finish() is None
    assert not slot.is_busy()
