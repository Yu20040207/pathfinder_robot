import pytest

from fwr_bringup.room_labels_core import RoomError, find_room, load_rooms


def test_parses_rooms_and_defaults_yaw():
    rooms = load_rooms('rooms:\n  - {name: A, x: 1.0, y: 2.0}\n  - {name: B, x: -1, y: 0, yaw: 1.57}\n')
    assert rooms[0] == {'name': 'A', 'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    assert rooms[1]['yaw'] == pytest.approx(1.57)
    assert find_room(rooms, 'B')['x'] == -1.0
    assert find_room(rooms, 'C') is None


def test_rejects_bad_documents():
    with pytest.raises(RoomError):
        load_rooms('other: []\n')
    with pytest.raises(RoomError):
        load_rooms('rooms:\n  - {x: 0, y: 0}\n')
    with pytest.raises(RoomError):
        load_rooms('rooms:\n  - {name: A, x: 0, y: 0}\n  - {name: A, x: 1, y: 1}\n')
    with pytest.raises(RoomError):
        load_rooms('rooms:\n  - {name: A, x: .nan, y: 0}\n')


def test_pose_checkpoint_serialises_the_pose():
    from fwr_bringup.pose_checkpoint import pose_yaml
    text = pose_yaml(-2.9351, -6.8909, -1.3136)
    assert 'x: -2.9351' in text and 'y: -6.8909' in text and 'yaw: -1.3136' in text
    import yaml
    assert yaml.safe_load(text) == {'x': -2.9351, 'y': -6.8909, 'yaw': -1.3136}


def test_ready_to_activate_gate():
    from fwr_bringup.auto_activate_core import ready_to_activate
    assert ready_to_activate(True, 0.2, True)
    assert not ready_to_activate(True, 0.2, False)     # no TF
    assert not ready_to_activate(False, 0.2, True)     # map never arrived
    assert not ready_to_activate(True, None, True)     # no scan yet
    assert not ready_to_activate(True, 9.9, True)      # stale scan
    # 锁存地图只发一次：收到后永远算就绪，不再看新鲜度。
    assert ready_to_activate(True, 0.01, True)


def test_refine_pose_recovers_a_shifted_prior():
    import math
    import numpy as np
    from fwr_bringup.auto_seed_core import refine_pose
    occ = np.zeros((40, 40), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    occ[10:13, 10:30] = True          # 一道内墙
    res, ox, oy = 0.05, -1.0, -1.0
    truth = (0.0, 0.0, 0.0)
    pts = []
    for ang in np.radians(np.arange(-180, 180, 3)):
        for r in np.arange(0.05, 2.5, 0.02):
            x = truth[0] + r * math.cos(truth[2] + ang)
            y = truth[1] + r * math.sin(truth[2] + ang)
            ix = int((x - ox) / res); iy = 39 - int((y - oy) / res)
            if not (0 <= ix < 40 and 0 <= iy < 40) or occ[iy, ix]:
                pts.append((r * math.cos(ang), r * math.sin(ang)))
                break
    score, x, y, yaw = refine_pose(occ, res, ox, oy, np.array(pts),
                                   0.20, -0.15, math.radians(6))
    assert score < 0.02, score
    assert abs(x) < 0.10 and abs(y) < 0.10, (x, y)
    assert abs(yaw) < math.radians(3), yaw


def test_heading_hold_corrects_drift_only_when_going_straight():
    from fwr_bringup.heading_hold import HeadingHold
    hold = HeadingHold(gain=1.5, limit_rad_s=.12)
    # 第一帧记录参考航向，不干预
    assert hold.update(.15, 0.0, 0.0, 1.0) == 0.0
    # 车头偏了 +0.1 rad → 给出负（回正）角速度
    assert hold.update(.15, 0.0, 0.1, 1.1) < 0.0
    # 偏得很多时被限幅
    assert abs(hold.update(.15, 0.0, 1.0, 1.2)) <= .12 + 1e-12
    # 有转向指令时完全不介入
    assert hold.update(.15, .3, 1.0, 1.3) == .3
    # 停车后回到原样
    assert hold.update(0.0, 0.0, 1.0, 1.4) == 0.0
    # 重新起步会以新航向为参考
    hold.update(.15, 0.0, 1.0, 2.0)
    assert abs(hold.update(.15, 0.0, 1.0, 2.1)) < 1e-9


def test_heading_hold_ignores_stale_or_nonfinite_input():
    from fwr_bringup.heading_hold import HeadingHold
    hold = HeadingHold()
    hold.update(.15, 0.0, 0.0, 1.0)
    # 停手很久后重新直行：重新取参考，不甩方向
    assert hold.update(.15, 0.0, .5, 5.0) == 0.0
    assert hold.update(.15, 0.0, float('nan'), 5.1) == 0.0
