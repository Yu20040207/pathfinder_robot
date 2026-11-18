"""转弯让位：要转大角度但车头贴太近 → 先直线后退一点腾出转弯空间。

2026-11-18 现场：车要掉头，`directional_safety_selector` 把前进改成原地转，
但车头已经在 `front_stop` 多边形里 → 碰撞监控把指令压成 0 → 车停住不动，
规划进度超时 → 目标失败。用户明确要求："直接后退一点让出位置也可以"。

与"死胡同沿轨迹倒车"的区别（两者共用倒车通道与车尾守卫）：
  · 触发：那个要**左右都窄**（原地掉头没位置）；这个只要**车头贴太近**，
    两侧可能很开阔（例如靠着墙掉头）。
  · 动作：那个**沿刚走过的轨迹**倒；这个只**直线后退一小段**。
  · 次数：那个同一目标点最多 2 次；这个一共最多 3 次（防止无限倒退）。
"""

from __future__ import annotations


def wants_backup(blocked_for_s: float,
                 raw_linear: float,
                 raw_angular: float,
                 front_min_m: float,
                 rear_min_m: float,
                 rear_blocked: bool,
                 attempts: int,
                 since_last_s: float,
                 blocked_s: float = 0.8,
                 turn_intent_radps: float = 0.15,
                 front_close_m: float = 0.45,
                 rear_need_m: float = 0.35,
                 max_attempts: int = 3,
                 cooldown_s: float = 3.0) -> tuple[bool, str]:
    """要不要为了转弯后退一点。返回 (是否后退, 原因)。"""
    if attempts >= max_attempts:
        return False, 'attempts-exhausted'
    if since_last_s < cooldown_s:
        return False, 'cooldown'
    if rear_blocked:
        return False, 'rear-blocked'
    if rear_min_m < rear_need_m:
        return False, 'rear-tight'
    # 必须是"想转弯"而不是"想直走"：直走被挡是死胡同那套逻辑管的
    if abs(raw_angular) < turn_intent_radps:
        return False, 'not-turn-intent'
    if blocked_for_s < blocked_s:
        return False, 'not-blocked-long-enough'
    # 只有**车头确实太近**才值得后退；否则后退也解决不了（可能是侧向被卡）
    if front_min_m > front_close_m:
        return False, 'front-not-close'
    return True, 'turn-blocked-front-close'


def backup_done(backed_m: float,
                elapsed_s: float,
                rear_min_m: float,
                rear_blocked: bool,
                dist_m: float = 0.20,
                max_s: float = 2.5,
                rear_stop_m: float = 0.20) -> tuple[bool, str]:
    """后退该停了吗。返回 (是否结束, 原因)。"""
    if rear_blocked:
        return True, 'rear-blocked'
    if rear_min_m < rear_stop_m:
        return True, 'rear-close'
    if backed_m >= dist_m:
        return True, 'reached'
    if elapsed_s >= max_s:
        return True, 'timeout'
    return False, ''
