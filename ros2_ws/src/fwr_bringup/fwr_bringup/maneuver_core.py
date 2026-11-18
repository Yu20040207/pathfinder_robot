"""让位决策：**执行前**判断"能不能就地转"，不能就后退，退到能转为止。

2026-11-18 用户定的规则（现场原话）：
    "监测到快碰撞的时候直接后退就可以了，后退到能转弯的过去了就好，
     还有种情况也就是左右两边都很接近，这样也是直接后退就好，
     就是在执行前就判断好是后退还是直接转弯就好。"

之前是三条独立分支（死胡同沿轨迹倒车 / 转弯短退 / 转弯净空守卫），
各用各的扇区和停止条件，互相打架 —— 一天内出了 5 次"该退不退 / 退 0.00 m"。
现在统一成一条规则：

    能转            → 不动（交给 Nav2 转）
    转不开/快撞且车尾有空间 → 后退
    后退的停止条件  = **又能转了**（与触发用同一套侧向量） / 退够 / 车尾贴住 / 超时

**触发量和停止量必须是同一套量**（这里是侧向左右净空和前方净空），
否则就会出现"一侧说转不开、另一侧说空间够了"的自相矛盾。
"""


def lateral_ok(lat_left_m: float, lat_right_m: float, need_m: float) -> bool:
    """左右两侧是否**都**够宽（能就地大角度转）。

    车身四角到中心 = |(0.30, 0.18)| = 0.35 m 是原地转的扫掠半径，
    need_m 默认 0.40（0.35 + 5 cm 余量；0.45 会白退，见 trail_reverser）。
    """
    return min(lat_left_m, lat_right_m) >= need_m


def can_turn(lat_left_m: float, lat_right_m: float, front_m: float,
             turn_need_m: float = 0.40, front_need_m: float = 0.45) -> bool:
    """现在就能就地转吗（不需要先退）。"""
    return (lateral_ok(lat_left_m, lat_right_m, turn_need_m)
            and front_m >= front_need_m)


def tight_turn_needed(*, raw_w_radps: float, lat_left_m: float, lat_right_m: float,
                      front_m: float, blocked_for_s: float,
                      turn_need_m: float = 0.40, w_min_radps: float = 0.40,
                      front_need_m: float = 1.30,
                      blocked_s: float = 0.5) -> bool:
    """是否"想大角度转但现在转不开"（= 死胡同掉头场景）。

    2026-11-18 现场（**过门口就后退**，回归事故）：
    原来的判据是"|ω|>=0.2 且两侧都 <0.40" —— 而**门口/走廊天然两侧都不够宽**
    （门洞 ~0.9 m，车宽 0.36 → 每侧约 0.27 m），Nav2 沿路径过门时又有曲率
    （|ω| 轻松 ≥0.2），于是每过一个门就触发让位后退。

    门口和死胡同的区别**不在两侧宽度**（两者都窄），而在：
      · 死胡同：前方被墙挡住（front 小），或者车已经被卡住不动了
      · 门口：前方是开阔的（front 很大），车正在穿过去

    所以必须再加一条"前方受限 或 确实卡住"，且要求**真正的大角度转向**。
    """
    if abs(raw_w_radps) < w_min_radps:
        return False
    if min(lat_left_m, lat_right_m) >= turn_need_m:
        return False
    return front_m < front_need_m or blocked_for_s >= blocked_s


def wants_backup(*, want_turn: bool, blocked_for_s: float, front_m: float,
                 lat_left_m: float, lat_right_m: float,
                 rear_min_m: float, rear_blocked: bool,
                 attempts: int, since_last_s: float,
                 turn_need_m: float = 0.40, front_need_m: float = 0.45,
                 rear_need_m: float = 0.55, blocked_s: float = 0.5,
                 max_attempts: int = 3,
                 cooldown_s: float = 3.0) -> tuple[bool, str]:
    """要不要后退。返回 (是否后退, 原因)。

    两条触发（用户明确要求的两种现场情况）：
      'turn-too-tight'      —— 想转但两侧（任一侧）不够宽
      'front-about-to-hit'  —— 直行/前进被挡够久，且车头已经很近
    两条都以 can_turn() 为前置：能转就绝不后退。
    """
    if can_turn(lat_left_m, lat_right_m, front_m, turn_need_m, front_need_m):
        return False, 'can-turn'
    if want_turn and not lateral_ok(lat_left_m, lat_right_m, turn_need_m):
        reason = 'turn-too-tight'
    elif front_m < front_need_m and blocked_for_s >= blocked_s:
        reason = 'front-about-to-hit'
    else:
        return False, 'no-need'
    if rear_blocked:
        return False, 'rear-blocked'
    if rear_min_m < rear_need_m:
        return False, 'rear-tight'
    if attempts >= max_attempts:
        return False, 'attempts-exhausted'
    if since_last_s < cooldown_s:
        return False, 'cooldown'
    return True, reason


def backup_done(*, backed_m: float, elapsed_s: float,
                lat_left_m: float, lat_right_m: float, front_m: float,
                rear_min_m: float, rear_blocked: bool,
                turn_need_m: float = 0.40, front_need_m: float = 0.45,
                dist_m: float = 0.80, max_s: float = 12.0,
                rear_stop_m: float = 0.20) -> tuple[bool, str]:
    """后退该停了吗。返回 (是否结束, 原因)。

    **第一条也是最重要的一条：又能转了就立刻停** —— 用户原话
    "后退到能转弯的过去了就好"。这里用的 can_turn() 和 wants_backup()
    的触发条件完全是同一套量，不可能再出现互相打脸。
    """
    if rear_blocked:
        return True, 'rear-blocked'
    if rear_min_m < rear_stop_m:
        return True, 'rear-close'
    if can_turn(lat_left_m, lat_right_m, front_m, turn_need_m, front_need_m):
        return True, 'can-turn-now'
    if backed_m >= dist_m:
        return True, 'reached'
    if elapsed_s >= max_s:
        return True, 'timeout'
    return False, ''


def accumulate_blocked(acc: float, dt: float, blocked: bool,
                       unblocked_s: float = 0.0, decay: float = 2.0,
                       grace_s: float = 0.4, cap: float = 5.0) -> float:
    """累计"被挡"时长，容忍 stop/slowdown 的快速抖动。

    碰撞监控会以 ~0.2 s 的周期在 stop 和放行之间来回切，如果一放行就把计时
    清零，永远攒不够 blocked_s（2026-11-18 实测原地转 2.9 s、3 次 STOP 却
    一次都没触发）。所以：短于 grace_s 的放行视为噪声不计；真正持续通畅
    （>= grace_s）才按 decay 倍速衰减。
    """
    if blocked:
        return min(cap, acc + dt)
    if unblocked_s < grace_s:
        return acc
    return max(0.0, acc - dt * decay)
