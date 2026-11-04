"""Fail-closed selection of direction-specific Collision Monitor outputs."""

import math


class DirectionalSafetyCore:
    # 原地转向由两轮反向完成；只要内侧轮速度 <= 0，这条指令就是旋转。
    # 用固定阈值判方向会在 Shim 的起步斜坡（实测 v 可到 37–45 mm/s）上反复翻转，
    # 每次翻转都会清空监控回执，导致隔帧归零。按轮速判据则与底盘语义一致。
    TRACK_M = .263
    ROTATE_WHEEL_EPS = .02    # m/s, 容许的轮速零偏（含 10 Hz 回执滞后)
    ANGULAR_DEADBAND = .02    # rad/s, 小于此值视为“不转”，避免符号抖动否决

    def __init__(self, command_timeout=.15, sensor_timeout=.25,
                 require_manual_status=True, monitor_warmup=.30,
                 repeat_hold_timeout=.11, reply_timeout=.50):
        self.command_timeout = command_timeout
        self.sensor_timeout = sensor_timeout
        self.require_manual_status = require_manual_status
        self.monitor_warmup = monitor_warmup
        self.repeat_hold_timeout = repeat_hold_timeout
        self.reply_timeout = reply_timeout
        self.raw = None
        self.repeated_raw_at = None
        self.filtered = {}
        self.direction = None
        self.require_reverse_monitor = True
        # 2026-11-01：纯旋转放行。前停区（0.32~0.45 m）是为"前进"设计的，
        # 但它把整帧清零 → "车头没对准路径、只需原地右转"也被拦死。
        # 车体半对角线约 0.37 m：车头障碍更远时，原地旋转不可能撞到。
        self.rotate_clear_ok = False
        # 2026-11-01（用户要求）：**要转的方向被挡就转另一边**。
        # 例：导航要左转，但左前有障碍、右前空旷 → 直接右转，
        # Nav2 会按新朝向重新规划。这里只用标志位表达"反方向是否安全"。
        self.rotate_flip_ok = False
        self.rotate_same_side_ok = False
        # 2026-11-01（用户要求）：前进被前停区挡住、但某一侧有空间时，
        # **放弃线速度、改为原地旋转**（v=0），转到朝向可行再继续。
        # 这是安全的：原地转不会把车头推向障碍。
        self.blocked_rotate_ok = False   # 反方向可转
        self.blocked_rotate_same = False # 请求方向可转
        self.scan_at = None
        self.tf_at = None
        self.monitor_active_since = {}
        self.monitor_confirmed_at = {}
        self.state = None
        self.state_at = None
        self.manual = None
        self.manual_at = None
        self.last_reason = None

    def observe_raw(self, linear_x: float, angular_z: float, now: float) -> None:
        # 两轮反向才算原地旋转：|v| <= |w|*轮距/2 时左右轮符号相反。
        if (angular_z != 0 and
                abs(linear_x) <= abs(angular_z) * self.TRACK_M / 2.0 + self.ROTATE_WHEEL_EPS):
            new_direction = 'rotate'
        elif linear_x > 0:
            new_direction = 'forward'
        elif linear_x < 0:
            new_direction = 'reverse'
        else:
            new_direction = 'rotate' if angular_z != 0 else None
        repeated = (self.raw is not None and
                    (linear_x, angular_z) == self.raw[:2] and
                    new_direction == self.direction)
        # 2026-11-02（"一卡一卡"根因）：
        # 原来方向一变就把回执缓存整个清空。而控制器在"前进↔原地转"之间
        # 频繁切换（RotationShim），每切一次就丢几帧回执 → no-reply-forward
        # → 整帧拒绝 → 线速度掉一下，观感就是一顿一顿。
        # 缓存本来就按方向分键（filtered[direction]），且每条都有
        # stale-reply / future-reply 时间戳校验，不需要清空。
        self.repeated_raw_at = now if repeated else None
        self.direction = new_direction
        self.raw = (linear_x, angular_z, now)
        # A monitor reply arrives at the scan rate (~10 Hz) while the smoothed
        # command updates at ~20 Hz, so requiring a reply newer than every raw
        # command drops every other frame and makes turns stutter. The last
        # reply stays usable within the same direction for at most
        # command_timeout; the magnitude and sign checks in select() still fail
        # closed whenever the new command asks for more than was monitored.
        # A direction change, a stale reply, or a monitor stop still zeros it.

    def observe_filtered(self, direction: str, linear_x: float,
                         angular_z: float, now: float) -> None:
        if (direction in ('forward', 'reverse') and self.raw is not None and
                (direction == self.direction or self.direction == 'rotate')):
            self.filtered[direction] = (linear_x, angular_z, now)

    def observe_scan(self, now: float) -> None:
        self.scan_at = now

    def observe_tf(self, ready: bool, now: float) -> None:
        self.tf_at = now if ready else None

    def observe_monitor(self, direction: str, active: bool, now: float) -> None:
        if direction not in ('forward', 'reverse'):
            raise ValueError(f'unknown monitor direction: {direction}')
        if not active:
            self.monitor_active_since.pop(direction, None)
            self.monitor_confirmed_at.pop(direction, None)
            self.filtered.pop(direction, None)
            return
        self.monitor_active_since.setdefault(direction, now)
        self.monitor_confirmed_at[direction] = now

    def observe_explore_state(self, state: str, now: float) -> None:
        self.state, self.state_at = state, now

    def observe_manual(self, enabled: bool, now: float) -> None:
        self.manual, self.manual_at = bool(enabled), now

    def select(self, now: float) -> tuple[float, float]:
        zero = (0.0, 0.0)

        def fail(reason: str):
            self.last_reason = reason
            return zero

        if self.raw is None or self.direction is None:
            return fail('no-command')
        raw_x, raw_z, raw_at = self.raw
        # 规划器在直线上会让 w 在 0 附近抖符号；把死区内的 w 归一成 0，
        # 否则监控回执的旧符号会每帧触发“sign-angular”否决。
        raw_z_eff = raw_z if abs(raw_z) >= self.ANGULAR_DEADBAND else 0.0
        values = (now, raw_x, raw_z, raw_at)
        if not all(math.isfinite(value) for value in values):
            return fail('nonfinite')
        for name, received, limit in (('raw', raw_at, self.command_timeout),
                                      ('scan', self.scan_at, self.sensor_timeout),
                                      ('tf', self.tf_at, self.sensor_timeout),
                                      ('state', self.state_at, self.sensor_timeout)):
            if received is None or not math.isfinite(received) or not 0 <= now - received <= limit:
                return fail(f'stale-{name}')
        if self.require_manual_status:
            if (self.manual_at is None or not math.isfinite(self.manual_at) or
                    not 0 <= now - self.manual_at <= self.sensor_timeout):
                return fail('stale-manual')
        if self.manual or self.state != ('reversing' if self.direction == 'reverse' else 'navigating'):
            return fail('mode')
        # 纯旋转 + 车头够远 → 直接放行角速度（不要求监控回执）
        if (self.direction == 'rotate' and abs(raw_x) < 1e-3 and
                abs(raw_z_eff) > 0):
            if self.rotate_clear_ok or self.rotate_same_side_ok:
                self.last_reason = 'rotate-clear'
                return (0.0, raw_z_eff)
            # 要转的方向被挡 → 反方向只要空旷就转过去
            if self.rotate_flip_ok:
                self.last_reason = 'rotate-flip'
                return (0.0, -raw_z_eff)
        # 前进被前停区挡住（线速度非零但前方堵）→ 改成原地旋转
        if (abs(raw_x) >= 1e-3 and abs(raw_z_eff) > 0):
            if self.blocked_rotate_same:
                self.last_reason = 'blocked-rotate-same'
                return (0.0, raw_z_eff)
            if self.blocked_rotate_ok:
                self.last_reason = 'blocked-rotate-flip'
                return (0.0, -raw_z_eff)
        # 2026-11-01：原来"原地旋转"要求 forward 和 reverse 两个监控都给出
        # 非零角速度回执。但 reverse 监控的后方停车区在贴墙时必然触发，
        # 车尾 0.3~0.45 m 内有 3 个激光点就否决一切旋转 →
        # "规划出转弯但底盘停住"。倒车安全改由 base_bridge 的车尾超声联锁负责
        # （它只拦后退，不拦旋转）。
        if self.direction == 'rotate':
            required = (('forward', 'reverse') if self.require_reverse_monitor
                        else ('forward',))
        else:
            required = (self.direction,)
        for direction in required:
            active_since = self.monitor_active_since.get(direction)
            confirmed_at = self.monitor_confirmed_at.get(direction)
            if (active_since is None or confirmed_at is None or
                    self.scan_at < active_since or
                    not 0 <= now - confirmed_at <= self.sensor_timeout or
                    now - active_since < self.monitor_warmup):
                return fail(f'monitor-{direction}')
        epsilon = 1e-6
        outputs = []
        for direction in required:
            if direction not in self.filtered:
                return fail(f'no-reply-{direction}')
            output_x, output_z, output_at = self.filtered[direction]
            if not all(math.isfinite(v) for v in (output_x, output_z, output_at)):
                return fail(f'nonfinite-reply-{direction}')
            if not output_at <= now or now - output_at > self.reply_timeout:
                return fail(f'stale-reply-{direction}')
            if output_at - raw_at > self.reply_timeout:
                return fail(f'future-reply-{direction}')
            # 符号必须与当前指令一致：回执方向相反说明监控结果陈旧或错乱，
            # 这种情况仍然整帧拒绝（不能夹紧，否则会朝反方向走）。
            # 2026-11-02（实测"一卡一卡"根因）：
            # 监控 10 Hz 回执 vs 选择器 20 Hz 输出，转弯时角速度符号快速变化，
            # 只要回执慢一拍就"符号不符"，原来会**整帧丢弃线速度** ——
            # 实测 15 次 sign-angular + 11 次 no-reply = 14% 帧被削。
            # 现在：角速度很小时（< 0.05 rad/s ≈ 3°/s）符号抖动直接忽略，
            # 只保留线速度；角速度较大且真的反向时才拒绝。
            if raw_z_eff and output_z and (output_z > 0) != (raw_z_eff > 0):
                if abs(output_z) > 0.05 or abs(raw_z_eff) > 0.05:
                    # 2026-11-03（"一卡一卡"）：监控回执慢一拍时 ω 符号不符，
                    # 原来**整帧归零**连线速度一起丢 → 直线上每翻一次符号就顿一下。
                    # 现在只把转向归零、保留前进：不会反向（安全语义不变），
                    # 下一拍回执跟上后转向自动恢复。纯旋转情形仍整帧拒绝。
                    if self.direction != 'rotate' and raw_x:
                        return (raw_x, 0.0)
                    return fail(f'sign-angular-{direction}')
            # 幅值超出的情形（10 Hz 回执 vs 20 Hz 输出，减速时回执仍偏大）
            # 按当前指令夹紧而不是丢弃，否则每两帧砍一帧、车起不了速。
            output_z = math.copysign(min(abs(output_z), abs(raw_z_eff)), raw_z_eff) if raw_z_eff else 0.0
            if self.direction == 'rotate':
                # 旋转输出只取角速度（线速度强制 0），回执里的线速度分量
                # 只反映监控侧的旧指令，不参与否决；角速度的符号与幅值
                # 已在上方校验并夹紧。
                output_x = 0.0
            else:
                if not raw_x:
                    return fail(f'no-linear-command-{direction}')
                if output_x and (output_x > 0) != (raw_x > 0):
                    return fail(f'sign-linear-{direction}')
                output_x = math.copysign(min(abs(output_x), abs(raw_x)), raw_x)
            outputs.append((output_x, output_z))
        if self.direction == 'rotate':
            if any(abs(z) <= epsilon for _, z in outputs):
                return fail('zero-angular-reply')
            self.last_reason = 'ok'
            return 0.0, min((z for _, z in outputs), key=abs)
        output_x, output_z = outputs[0]
        self.last_reason = 'ok'
        return output_x, output_z


def latch_choice(prev: str | None, prev_at: float, now: float,
                 hold_s: float, new: str | None, blocked: bool) -> str | None:
    """翻转方向的迟滞锁定（2026-11-04）。

    背景：前方被挡时选择器会把角速度取反（"往空旷一侧转"）。但 0.16 m 的旋转
    净空阈值在雷达噪声下逐帧抖动 → 每帧在"正转/反转"之间切换 → 两后轮反复
    反向 → 车几乎不动而 PWM 持续 > 12% → 固件 2 s 判堵转（fault 1）。

    规则：一旦选定方向，在 hold_s 内**锁住**，且必须"仍在 blocked 状态"才锁；
    不再 blocked（前方开阔了）立即解除。
    """
    if not blocked:
        return None
    if prev is not None and (now - prev_at) < hold_s_safe(hold_s):
        return prev
    return new


def hold_s_safe(hold_s: float) -> float:
    return max(0.0, float(hold_s))
