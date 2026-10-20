from types import SimpleNamespace as NS
from fwr_bringup.recent_track import RecentTrack
from fwr_bringup.backtrack_core import BacktrackController


def pose(x, y=1., yaw=0.):
    return NS(header=NS(frame_id='map'), pose=NS(pose=NS(position=NS(x=x, y=y),
              orientation=NS(x=0., y=0., z=0., w=1.))))


def grid(block=None):
    rows = [0] * (60 * 30)
    if block is not None:
        rows[block] = 100
    return NS(header=NS(frame_id='map'), info=NS(width=60, height=30, resolution=.05,
           origin=NS(position=NS(x=0., y=0.))), data=rows)


def track():
    t = RecentTrack()
    for i in range(20):
        t.observe(pose(1. + .02 * i), i * .1)
    return t


def test_one_attempt_segment_and_total_budget():
    c = BacktrackController()
    t = track()
    assert c.begin(t, grid(), 2.)
    assert c.segment_limit_m <= .1 and c.total_limit_m <= .3
    cmd, status = c.update(pose(1.38), grid(), True, 0, 2.05)
    assert status == 'running' and -.08 <= cmd.linear.x < 0
    assert not c.begin(t, grid(), 3.)


def test_unknown_and_occupied_footprint_abort():
    t = track()
    m = grid()
    c = BacktrackController()
    assert c.begin(t, m, 2.)
    m.data[20 * 60 + 27] = -1
    cmd, status = c.update(pose(1.38), m, True, 0, 2.1)
    assert status == 'aborted' and cmd.linear.x == 0


def test_stale_fault_jump_no_progress_timeout_and_cancel_stop():
    for case in ('stale', 'fault', 'jump', 'timeout', 'cancel'):
        c = BacktrackController()
        assert c.begin(track(), grid(), 2.)
        if case == 'cancel':
            c.cancel()
        cmd, status = c.update(pose(2. if case == 'jump' else 1.38), grid(),
                               case != 'stale', 1 if case == 'fault' else 0,
                               13. if case == 'timeout' else 2.1)
        assert status == 'aborted' and cmd.linear.x == 0
    c = BacktrackController()
    assert c.begin(track(), grid(), 2.)
    c.update(pose(1.38), grid(), True, 0, 2.1)
    cmd, status = c.update(pose(1.38), grid(), True, 0, 3.5)
    assert status == 'aborted' and cmd.linear.x == 0
