"""后退让位核心逻辑的单元测试（纯 numpy/数学，不依赖 ROS）。"""
import math

from fwr_bringup import trail_reverser_core as tc


def test_trail_records_with_step_and_keeps_window():
    trail = tc.TrailBuffer(step_m=0.1, max_len_m=0.5)
    assert trail.append(0.0, 0.0) is True
    assert trail.append(0.02, 0.0) is False        # 不足一步，不记录
    assert trail.append(0.12, 0.0) is True
    # 走很远，窗口只保留 max_len_m
    for i in range(1, 30):
        trail.append(0.12 + i * 0.1, 0.0)
    assert trail.length() <= 0.5 + 0.11


def test_target_behind_follows_the_path():
    trail = tc.TrailBuffer(step_m=0.1, max_len_m=5.0)
    for i in range(11):
        trail.append(i * 0.1, 0.0)                 # 沿 x 轴走 1 m
    tgt = trail.target_behind(0.4)
    assert tgt is not None
    # 轨迹点间隔 0.1 m，退 0.4 m 落在 0.6 附近（允许一个采样间隔）
    assert abs(tgt[0] - 0.6) <= 0.11, '应该退到身后 0.4 m 附近'
    # 轨迹不够长 → 返回最老的点
    short = tc.TrailBuffer(step_m=0.1)
    short.append(0.0, 0.0)
    short.append(0.1, 0.0)
    assert short.target_behind(5.0) == (0.0, 0.0)


def test_reverse_steering_points_the_tail_at_target():
    # 车朝 +x（yaw=0），目标在身后（-x 方向）→ 车尾已对准 → 不需要转
    w = tc.reverse_steering(0.0, 0.0, 0.0, -1.0, 0.0)
    assert abs(w) < 1e-9
    # 目标在"正后方偏左"：车尾要从 180° 转到 162°，也就是车要顺时针转 →
    # 角速度应为负（这里曾经把期望写成正号，属于测试写错，不是代码错）
    w2 = tc.reverse_steering(0.0, 0.0, 0.0, -1.0, 0.3)
    assert w2 < 0.0
    assert abs(w2) <= 0.5


def test_trigger_requires_blocked_and_tight_sides():
    ok, why = tc.should_start(blocked_for_s=2.0, side_left_m=0.3, side_right_m=0.3, rear_blocked=False,
                              trail_len_m=2.0, since_last_s=10.0)
    assert ok and why == 'start'
    # 侧面够宽 → 不需要后退让位
    assert tc.should_start(blocked_for_s=2.0, side_left_m=0.8, side_right_m=0.8, rear_blocked=False,
                           trail_len_m=2.0, since_last_s=10.0)[0] is False
    # 车尾被挡 → 绝不后退
    assert tc.should_start(blocked_for_s=9.0, side_left_m=0.1, side_right_m=0.1, rear_blocked=True,
                           trail_len_m=3.0, since_last_s=99.0) == (False, 'rear-blocked')
    # 轨迹太短 / 冷却中
    assert tc.should_start(blocked_for_s=9.0, side_left_m=0.1, side_right_m=0.1, rear_blocked=False,
                           trail_len_m=0.2, since_last_s=99.0)[0] is False
    assert tc.should_start(blocked_for_s=9.0, side_left_m=0.1, side_right_m=0.1, rear_blocked=False,
                           trail_len_m=3.0, since_last_s=1.0)[0] is False


def test_stop_when_both_sides_have_room():
    # 主要条件：左右都够 → 停
    assert tc.should_stop(side_left_m=0.95, side_right_m=0.95, rear_blocked=False, reversed_m=0.3,
                          elapsed_s=3.0) == (True, 'enough-room')
    # 还在退
    assert tc.should_stop(side_left_m=0.3, side_right_m=0.3, rear_blocked=False, reversed_m=0.3,
                          elapsed_s=3.0) == (False, 'reversing')
    # 车尾被挡 → 立刻停
    assert tc.should_stop(side_left_m=0.3, side_right_m=0.3, rear_blocked=True, reversed_m=0.1,
                          elapsed_s=1.0) == (True, 'rear-blocked')
    # 安全上限
    assert tc.should_stop(side_left_m=0.3, side_right_m=0.3, rear_blocked=False, reversed_m=1.3,
                          elapsed_s=3.0) == (True, 'max-distance')
    assert tc.should_stop(side_left_m=0.3, side_right_m=0.3, rear_blocked=False, reversed_m=0.2,
                          elapsed_s=11.0) == (True, 'timeout')


def test_one_open_side_is_enough_to_turn_no_reverse():
    """一侧很空、另一侧贴墙 → 不该后退（现场 bug：右边能右转却开始后退）。

    原实现把左右扇区混在一起取 min：左 0.3 / 右 1.0 → 0.3 → 误判"两侧都窄"。
    正确语义：只有 max(左,右) 都小于阈值才需要后退。
    """
    ok, why = tc.should_start(blocked_for_s=5.0, side_left_m=0.3, side_right_m=1.0,
                              rear_blocked=False, trail_len_m=3.0, since_last_s=99.0)
    assert (ok, why) == (False, 'side-clear'), '右边有 1 m 空间时不该后退'


def test_stop_reversing_as_soon_as_one_side_is_wide():
    """后退过程中任意一侧变宽 ≥ turn_clear_m 就结束（有一边能转就够了）。"""
    assert tc.should_stop(side_left_m=0.2, side_right_m=0.95, rear_blocked=False,
                          reversed_m=0.2, elapsed_s=1.0) == (True, 'enough-room')


def test_reverse_pause_skips_waypoint_after_limit():
    """让位暂停/恢复/跳点的语义（用户要求：同一目标点最多让位 2 次，之后跳过）。"""
    from fwr_bringup.exploration_core import ExplorationStateMachine

    sm = ExplorationStateMachine()
    sm.start_fixed(legs=3)
    assert sm.fixed_leg_index == 0
    assert sm.skip_leg() is True
    assert sm.fixed_leg_index == 1, '跳过一次应指向第 2 段'
    assert sm.skip_leg() is True
    assert sm.fixed_leg_index == 2
    assert sm.skip_leg() is False, '最后一段跳过后没有后续'
    assert sm.fixed_leg_index >= 3, '越过最后一段后不再指向有效段'


def test_resume_through_poses_redispatches_instead_of_stopping():
    """一次穿过模式：停止→继续必须重新派发，不能因为 dispatched>=legs 直接结束。

    现场：面板显示"已继续：继续多点路线（剩余 6 点）"，但车不动，
    日志里 enabled=True 后立刻 enabled=False（状态机判定已走完）。
    """
    from fwr_bringup.exploration_core import ExplorationStateMachine

    sm = ExplorationStateMachine()
    sm.start_fixed(legs=1)
    # 派发一次
    d = sm.update(True, True, 0, 'idle', 0.0)
    assert d.action == 'dispatch'
    # 跑到一半被停止
    sm.stop()
    sm.update(True, True, 0, 'canceled', 1.0)
    # 继续
    assert sm.resume_fixed() is True
    sm.reset_dispatched()
    d = sm.update(True, True, 0, 'idle', 2.0)
    assert d.action == 'dispatch', '继续后应重新派发，而不是停止'
