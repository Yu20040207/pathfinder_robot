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
    ok, why = tc.should_start(blocked_for_s=2.0, side_min_m=0.3, rear_blocked=False,
                              trail_len_m=2.0, since_last_s=10.0)
    assert ok and why == 'start'
    # 侧面够宽 → 不需要后退让位
    assert tc.should_start(blocked_for_s=2.0, side_min_m=0.8, rear_blocked=False,
                           trail_len_m=2.0, since_last_s=10.0)[0] is False
    # 车尾被挡 → 绝不后退
    assert tc.should_start(blocked_for_s=9.0, side_min_m=0.1, rear_blocked=True,
                           trail_len_m=3.0, since_last_s=99.0) == (False, 'rear-blocked')
    # 轨迹太短 / 冷却中
    assert tc.should_start(blocked_for_s=9.0, side_min_m=0.1, rear_blocked=False,
                           trail_len_m=0.2, since_last_s=99.0)[0] is False
    assert tc.should_start(blocked_for_s=9.0, side_min_m=0.1, rear_blocked=False,
                           trail_len_m=3.0, since_last_s=1.0)[0] is False


def test_stop_when_both_sides_have_room():
    # 主要条件：左右都够 → 停
    assert tc.should_stop(side_min_m=0.95, rear_blocked=False, reversed_m=0.3,
                          elapsed_s=3.0) == (True, 'enough-room')
    # 还在退
    assert tc.should_stop(side_min_m=0.3, rear_blocked=False, reversed_m=0.3,
                          elapsed_s=3.0) == (False, 'reversing')
    # 车尾被挡 → 立刻停
    assert tc.should_stop(side_min_m=0.3, rear_blocked=True, reversed_m=0.1,
                          elapsed_s=1.0) == (True, 'rear-blocked')
    # 安全上限
    assert tc.should_stop(side_min_m=0.3, rear_blocked=False, reversed_m=1.3,
                          elapsed_s=3.0) == (True, 'max-distance')
    assert tc.should_stop(side_min_m=0.3, rear_blocked=False, reversed_m=0.2,
                          elapsed_s=11.0) == (True, 'timeout')
