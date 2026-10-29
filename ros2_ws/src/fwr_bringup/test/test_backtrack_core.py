from types import SimpleNamespace as NS
from fwr_bringup.recent_track import RecentTrack
from fwr_bringup.backtrack_core import BacktrackController, _clear, _clear_problem
from fwr_bringup.base_bridge_core import RecoveryPermitCore


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


def test_recent_track_survives_nav2_progress_timeout_while_stationary():
    t = RecentTrack()
    for i in range(11):
        t.observe(pose(1. + .01 * i), i * .1)
    # Nav2 may wait 15 s before declaring a blocked goal. Odometry continues
    # arriving during that wait, but the robot remains at the same pose.
    for i in range(1, 81):
        t.observe(pose(1.1), 1. + i * .2)
    assert len(t.points) >= 2
    assert t.points[0].x <= 1.05
    assert t.points[-1].x == 1.1


def test_recent_track_accumulates_small_odom_steps():
    t = RecentTrack()
    for i in range(41):
        t.observe(pose(1. + i * .003), i * .05)
    assert len(t.points) >= 9
    assert t.points[-1].x >= 1.11


def test_one_transform_preserves_track_when_slam_corrects_map_frame():
    t = RecentTrack()
    for i in range(13):
        sample = pose(1. + i * .02, y=.7)
        sample.header.frame_id = 'odom'
        t.observe(sample, i * .1)
    # A 0.18 m map correction between samples must not become a false step
    # along the driven trajectory when a single current transform is applied.
    mapped = t.transformed('map', .18, -.12, .2)
    assert mapped.frame_id == 'map'
    assert len(mapped.points) == len(t.points)
    assert all(abs(((b.x-a.x)**2 + (b.y-a.y)**2)**.5 - .02) < 1e-8
               for a, b in zip(mapped.points, list(mapped.points)[1:]))
    assert BacktrackController().begin(mapped, grid(), 1.3)


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


def test_recovery_refusal_reports_unsafe_map():
    m = grid(block=20 * 60 + 27)
    c = BacktrackController()
    assert not c.begin(track(), m, 2.)
    assert c.last_refusal.startswith('recorded path footprint not known free (occupied at ')


def test_recovery_uses_only_known_free_suffix_of_recorded_path():
    m = grid()
    # Latest robot footprint is clear, but the older path crosses an
    # unknown cell. A short reverse may use the safe suffix only.
    m.data[20 * 60 + 16] = -1
    c = BacktrackController()
    assert c.begin(track(), m, 2.)
    assert .05 <= sum(((b.x-a.x)**2 + (b.y-a.y)**2)**.5
                        for a, b in zip(c._path, c._path[1:])) < .30
    assert all(_clear(m, p.x, p.y) for p in c._path)


def test_recovery_refusal_reports_discontinuity_size_and_endpoints():
    t = RecentTrack()
    for i in range(7):
        t.observe(pose(1. + i * .02), i * .1)
    t.observe(pose(1.25), .7)
    c = BacktrackController()
    assert not c.begin(t, grid(), .8)
    assert c.last_refusal == 'track discontinuity 0.130 m from (1.25,1.00) to (1.12,1.00)'


def test_clearance_diagnostic_distinguishes_unknown_and_occupied():
    m = grid()
    index = 20 * 60 + 27
    m.data[index] = -1
    assert _clear_problem(m, 1.38, 1.) == 'unknown'
    m.data[index] = 100
    assert _clear_problem(m, 1.38, 1.) == 'occupied'


def test_local_inflation_cost_is_allowed_but_inscribed_obstacle_is_not():
    c = BacktrackController()
    m = grid()
    assert c.begin(track(), m, 2.)
    local = grid()
    local.data[20 * 60 + 27] = 50
    cmd, status = c.update(pose(1.38), m, True, 0, 2.05,
                           local_grid=local, map_to_local=lambda x, y: (x, y))
    assert status == 'running' and cmd.linear.x < 0
    local.data[20 * 60 + 27] = 99
    cmd, status = c.update(pose(1.38), m, True, 0, 2.10,
                           local_grid=local, map_to_local=lambda x, y: (x, y))
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


def test_new_local_obstacle_aborts_even_if_old_global_map_is_free():
    c = BacktrackController()
    m = grid()
    assert c.begin(track(), m, 2.)
    local = grid(block=20 * 60 + 27)
    cmd, status = c.update(pose(1.38), m, True, 0, 2.1,
                           local_grid=local, map_to_local=lambda x, y: (x, y))
    assert status == 'aborted' and cmd.linear.x == 0


def test_heading_independent_swept_clearance_rejects_corner_obstacle():
    m = grid()
    # 0.35 m sideways from the centre is outside the unrotated width,
    # yet inside a rotated chassis corner's swept circle.
    m.data[27 * 60 + 27] = 100
    assert not _clear(m, 1.375, 1.0)


def test_three_bounded_reverse_permits_follow_recorded_path():
    controller = BacktrackController()
    permit = RecoveryPermitCore()
    m = grid()
    assert controller.begin(track(), m, 2.)
    assert permit.request(True, 1, 2., True, True, True)[0]
    segment_count = 1
    last_x = 1.38
    final_status = None
    for n in range(1, 16):
        x = round(1.38 - .02 * n, 2)
        permit.observe_distance(last_x - x)
        cmd, status = controller.update(pose(x), m, True, 0, 2. + .1 * n)
        last_x = x
        if status == 'segment_done':
            assert cmd.linear.x == 0
            assert permit.request(False, 1, 2. + .1 * n, True, True, True)[0]
            assert permit.request(True, 1, 2. + .1 * n, True, True, True)[0]
            controller.next_segment(2. + .1 * n)
            segment_count += 1
        elif status == 'done':
            final_status = status
            break
        else:
            assert status == 'running'
        assert permit.total_m <= .30
    assert final_status == 'done'
    assert 2 <= segment_count <= 3
    assert permit.total_m <= .30
