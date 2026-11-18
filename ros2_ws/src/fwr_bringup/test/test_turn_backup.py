"""转弯让位：要转大角度但车头贴太近 → 直线后退一点腾空间。

2026-11-18 现场：车要掉头，前进被 front_stop 压零 → 改成原地转又被压零
→ 车停住 → 规划进度超时 → 目标失败。用户要求"后退一点让出位置"。
"""
from fwr_bringup.turn_backup_core import backup_done, wants_backup

BASE = dict(blocked_for_s=1.5, raw_linear=0.05, raw_angular=0.55,
            front_min_m=0.32, rear_min_m=0.80, rear_blocked=False,
            attempts=0, since_last_s=10.0)


def test_backs_up_when_front_close_and_turning():
    ok, why = wants_backup(**BASE)
    assert ok and why == 'turn-blocked-front-close'


def test_no_backup_for_straight_motion():
    """想直走被挡是死胡同那套逻辑管的，这里不能抢。"""
    ok, why = wants_backup(**{**BASE, 'raw_angular': 0.02})
    assert not ok and why == 'not-turn-intent'


def test_no_backup_when_front_is_far():
    """车头不近就后退也解决不了（可能是侧向被卡）。"""
    ok, why = wants_backup(**{**BASE, 'front_min_m': 1.2})
    assert not ok and why == 'front-not-close'


def test_no_backup_when_rear_blocked():
    ok, why = wants_backup(**{**BASE, 'rear_blocked': True})
    assert not ok and why == 'rear-blocked'


def test_no_backup_when_rear_tight():
    ok, why = wants_backup(**{**BASE, 'rear_min_m': 0.2})
    assert not ok and why == 'rear-tight'


def test_attempt_limit_and_cooldown():
    ok, _ = wants_backup(**{**BASE, 'attempts': 3})
    assert not ok, '不能无限倒退'
    ok, why = wants_backup(**{**BASE, 'since_last_s': 0.5})
    assert not ok and why == 'cooldown'


def test_backup_stop_conditions():
    assert backup_done(0.05, 0.5, 1.0, False, front_min_m=0.10)[1] == ''
    assert backup_done(0.65, 5.0, 1.0, False, front_min_m=0.10) == (True, 'reached')
    assert backup_done(0.05, 9.0, 1.0, False, front_min_m=0.10) == (True, 'timeout')
    assert backup_done(0.05, 0.2, 1.0, True, front_min_m=0.10) == (True, 'rear-blocked')
    assert backup_done(0.05, 0.2, 0.10, False, front_min_m=0.10) == (True, 'rear-close')


def test_stops_when_front_has_room_again():
    """现场教训：固定退 0.20 m 不够 —— 车头 0.09 m 只退到 0.26 m，
    仍在 front_stop(0.32~0.38) 里，照样转不了。必须退到真腾出空间。"""
    done, why = backup_done(0.30, 3.0, 1.0, False, front_min_m=0.58,
                            front_clear_m=0.55)
    assert done and why == 'front-clear'


def test_keeps_backing_while_front_still_close():
    done, why = backup_done(0.30, 3.0, 1.0, False, front_min_m=0.26,
                            front_clear_m=0.55)
    assert not done and why == '', '车头 0.26 m 还在 front_stop 里，要继续退'


def test_defaults_allow_enough_distance():
    """默认上限必须能覆盖"从 0.09 m 退到 0.55 m"（至少 0.46 m）。"""
    import inspect
    sig = inspect.signature(backup_done)
    assert sig.parameters['dist_m'].default >= 0.46
    assert sig.parameters['front_clear_m'].default >= 0.45


def test_distance_limit_is_bounded():
    """后退要有上限（防失控），但要够用。

    2026-11-18：原来定 0.20 m，现场车头 0.09 m 退完还剩 0.26 m，
    仍在 front_stop 里 → 白退一次。上限放到 0.60 m（0.09→0.55 只需 0.46 m），
    同时仍然有明确上限；真正的安全兜底是车尾守卫和后雷达。
    """
    import inspect
    sig = inspect.signature(backup_done)
    assert 0.30 <= sig.parameters['dist_m'].default <= 1.00


def test_explorer_does_not_abort_task_during_reverse():
    """让位期间探索器不能把任务杀掉。

    2026-11-18 现场：让位节点打开 manual_enabled → frontier_explorer._on_manual
    当成"人接管"直接 _trip() 中止整个任务 → 让位结束"恢复任务"时任务已死
    → 表现"后退了但没有继续导航"。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' /
           'frontier_explorer.py').read_text()
    i = src.index('def _on_manual')
    block = src[i:i + 1600]
    # 注意用带 self. 的真实调用去匹配：注释里也写过 _trip() 字样，会误命中
    assert block.index('self._reverse_active') < block.index('self._trip('), \
        '必须先判断让位状态，再决定是否中止'
    assert block.index('self._reverse_grace_until') < block.index('self._sm.stop()'), \
        '停状态机必须在让位/宽限期判断之后'


def test_rearm_is_retried():
    """现场：base_bridge 回 `arm 失败：reason=serial not ready`。

    倒车刚结束时底盘串口可能还在恢复，原来只调一次就放弃 → 导航再也接不上。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' /
           'trail_reverser.py').read_text()
    assert '_arm_done' in src, '必须有回执处理'
    assert '_rearm_tries' in src and '_rearm_deadline' in src, '必须带重试与总时限'
    i = src.index('def _arm_done')
    block = src[i:i + 1500]
    assert '_rearm_at = time.monotonic() + 0.7' in block, '失败后要重新排期重试'
    assert 'res.success' in block, '必须检查服务回执，不能盲发'


def test_front_stop_margin_exceeds_braking_distance():
    """front_stop 的安全余量必须大于刹车距离（2026-11-18 撞墙事故）。

    现场：车体前缘 0.30 m，front_stop 原来从 0.32 起 → 余量只有 2 cm；
    而 0.16~0.20 m/s、减速度 0.35 m/s² 的刹车距离是 3.7~5.7 cm，
    再加 10 Hz 扫描的延迟 —— 物理上必然蹭上。
    现在要求余量 >= 0.08 m（覆盖 0.20 m/s 的 5.7 cm + 延迟）。
    """
    import yaml
    from pathlib import Path
    cfg = Path(__file__).resolve().parents[1] / 'config' / 'real_collision_monitor_forward.yaml'
    fwd = yaml.safe_load(cfg.read_text())['collision_monitor_forward']['ros__parameters']
    pts = fwd['front_stop']['points']
    xs = pts[0::2]
    ys = pts[1::2]
    margin = min(xs) - 0.30          # 车体前缘在 x=+0.30
    assert margin >= 0.08, f'front_stop 余量只有 {margin:.2f} m，不足以刹停'
    assert 0.22 <= max(ys) <= 0.26, '半宽必须保持 0.22（门框不误触发的下限）'
    assert fwd['front_stop']['action_type'] == 'stop'


def test_reverse_episode_counter_resets_on_progress():
    """让位计数必须会复位，否则累计 3 次后每次让位都直接跳过该段。

    2026-11-18 现场 bug：_reverse_episodes 只有 +=1，从来没有 =0。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' /
           'frontier_explorer.py').read_text()
    resets = [ln for ln in src.splitlines()
              if 'self._reverse_episodes = 0' in ln]
    assert len(resets) >= 2, '至少要在"目标成功"和"换段"两处复位'


def test_trail_reverse_takes_priority_over_short_backup():
    """死胡同里必须优先"沿轨迹倒车"，而不是只退 0.33 m 的短退。

    2026-11-18 现场：短退排在前面抢着执行，车尾只剩 0.40 m 时退 0.33 m 就
    rear-close，既没腾出转弯空间又先贴上后墙。用户原话"只是后退了一下，
    没退到可以够位置转弯"。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' /
           'trail_reverser.py').read_text()
    i = src.index('def _maybe_start')
    block = src[i:i + 3000]
    assert block.index('tc.should_start(') < block.index('self._try_turn_backup('), \
        '沿轨迹倒车必须排在短退之前'


def test_short_backup_needs_rear_room():
    """短退起步前必须确认车尾够退，否则会先贴上后墙。"""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' /
           'trail_reverser.py').read_text()
    i = src.index('def _try_turn_backup')
    block = src[i:i + 900]
    assert "turn_rear_need_m" in block, '必须检查车尾空间'


def test_blocked_time_accumulates_through_toggling():
    """碰撞监控抖动时也要攒够"被挡"时长（2026-11-18 旋转撞墙的根因）。

    现场：原地旋转时 stop/slowdown 每 0.2 s 交替一次，连续计时器每次被清零，
    2.9 s 里一次都没攒到 0.8 s → 车一直转，最后车角蹭上墙。
    """
    from fwr_bringup.turn_backup_core import accumulate_blocked
    acc = 0.0
    for _ in range(8):
        acc = accumulate_blocked(acc, 0.2, True, unblocked_s=0.0)
        acc = accumulate_blocked(acc, 0.2, False, unblocked_s=0.2)   # 短暂放行=噪声
    assert acc >= 0.5, f'抖动 3.2 s 后应攒够阈值，实际 {acc:.2f}'


def test_blocked_time_decays_when_clear():
    from fwr_bringup.turn_backup_core import accumulate_blocked
    acc = accumulate_blocked(0.8, 1.0, False, unblocked_s=1.0)
    assert acc == 0.0, '连续 1 s 畅通应当衰减到 0'
    acc = accumulate_blocked(0.0, 100.0, True, unblocked_s=0.0)
    assert acc == 5.0, '必须有上限，避免长时间累加后一直触发'


def test_turn_tight_bypasses_side_clear_requirement():
    """转弯净空不足时，即使两侧不是"都窄"也要允许沿轨迹倒车。

    2026-11-18 现场：两侧 0.6~0.8 m（不满足"都 <0.5 m"），车头还有 0.20 m，
    可是原地大角度转弯照样把车角蹭到墙（车角扫掠半径 0.35 m）。
    """
    from fwr_bringup.trail_reverser_core import should_start
    base = dict(side_left_m=0.8, side_right_m=0.7, rear_blocked=False,
                trail_len_m=3.0, since_last_s=99.0, blocked_for_s=0.0)
    ok, why = should_start(**base)
    assert not ok and why == 'side-clear', '两侧开阔时不该退'
    ok, why = should_start(**base, turn_tight=True)
    assert ok, f'转弯净空不足必须能触发倒车，实际 {why}'
    # 车尾被堵时依然禁止
    ok, why = should_start(**{**base, 'rear_blocked': True}, turn_tight=True)
    assert not ok and why == 'rear-blocked'


def test_turn_guard_requires_turning_and_tight_sides():
    """守卫必须同时满足"在转弯"和"侧向窄"，直行过门不能误触发。"""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' /
           'trail_reverser.py').read_text()
    i = src.index('def _tight_turn_guard')
    block = src[i:i + 1800]
    assert 'angular.z) >= radps' in block, '必须要求角速度够大'
    assert '_lateral_clearance() < need' in block, '必须要求侧向净空不足'
    assert 'turn_max_attempts' in block, '必须有次数上限，防止反复退'


def test_dispatch_does_not_kill_route_on_transient_tf_gap():
    """一帧拿不到 map→base 的 TF 不能把整条路线停掉（2026-11-18 现场）。

    现象：让位结束"恢复任务"后 0.17 s，enabled 变 False、路线消失 ——
    因为 _dispatch 里 mapped is None 时直接 self._sm.stop()，而且静默。
    """
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'fwr_bringup' /
           'frontier_explorer.py').read_text()
    lines = src.splitlines()
    disp = next(i for i, ln in enumerate(lines)
                if ln.strip().startswith('def _dispatch'))
    start = next(i for i in range(disp, len(lines))
                 if 'if mapped is None:' in lines[i])
    body = []
    for ln in lines[start + 1:]:
        if ln.strip() and len(ln) - len(ln.lstrip()) <= 8:
            break                      # 缩进回到方法级别 = if 块结束
        body.append(ln)
    code = [ln for ln in body if not ln.strip().startswith('#')]
    block = '\n'.join(code)
    assert 'self._sm.stop()' not in block, 'TF 缺失时不许停状态机'
    assert '下一帧再试' in block, '应只跳过本帧并限流告警'
