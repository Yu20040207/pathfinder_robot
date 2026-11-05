"""沿"原路"后退让位的纯逻辑（不依赖 ROS，可单元测试）。

用户 2026-11-05 的要求：
  · 后退时**沿原路返回**（车刚开过的路一定是通的），而不是盲目直退
  · 超声只用来确认"退的路上有没有新东西挡住"
  · 单次后退**不设固定距离**，退到左右都有足够位置转弯为止
  · 触发条件：前方被挡 且 左右都 < 0.5 m

这里只放数学/状态判断；ROS 节点见 trail_reverser.py。
"""
from __future__ import annotations

import math


class TrailBuffer:
    """记录机器人走过的轨迹（按距离抽稀），并能取"身后 N 米"的目标点。"""

    def __init__(self, step_m: float = 0.05, max_len_m: float = 5.0) -> None:
        self.step_m = max(0.005, float(step_m))
        self.max_len_m = max(self.step_m, float(max_len_m))
        self._pts: list[tuple[float, float]] = []

    def append(self, x: float, y: float) -> bool:
        """新位姿；与上一个保存点距离 >= step_m 才记录。返回是否记录。"""
        if not self._pts:
            self._pts.append((float(x), float(y)))
            return True
        px, py = self._pts[-1]
        if math.hypot(x - px, y - py) < self.step_m - 1e-9:
            return False
        self._pts.append((float(x), float(y)))
        self._trim()
        return True

    def _trim(self) -> None:
        total = 0.0
        for i in range(len(self._pts) - 1, 0, -1):
            total += math.hypot(self._pts[i][0] - self._pts[i - 1][0],
                                self._pts[i][1] - self._pts[i - 1][1])
            if total > self.max_len_m:
                self._pts = self._pts[i:]
                return

    @property
    def points(self) -> list[tuple[float, float]]:
        return list(self._pts)

    def length(self) -> float:
        return sum(math.hypot(self._pts[i][0] - self._pts[i - 1][0],
                              self._pts[i][1] - self._pts[i - 1][1])
                   for i in range(1, len(self._pts)))

    def target_behind(self, back_m: float) -> tuple[float, float] | None:
        """沿轨迹往回退 back_m 处的点（不足则返回最老的点）。"""
        if not self._pts:
            return None
        total = 0.0
        for i in range(len(self._pts) - 1, 0, -1):
            seg = math.hypot(self._pts[i][0] - self._pts[i - 1][0],
                             self._pts[i][1] - self._pts[i - 1][1])
            if total + seg >= back_m:
                return self._pts[i - 1]
            total += seg
        return self._pts[0]

    def clear(self) -> None:
        self._pts.clear()


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def reverse_steering(x: float, y: float, yaw: float,
                     tx: float, ty: float,
                     kp: float = 1.5, max_w: float = 0.5) -> float:
    """倒退时朝目标点转的角速度。

    倒退跟车头朝向的关系：**车尾要对准目标**，所以期望朝向 = 指向目标的角度 + π。
    """
    desired = math.atan2(ty - y, tx - x) + math.pi
    err = normalize_angle(desired - yaw)
    return max(-max_w, min(max_w, kp * err))


def should_start(*, blocked_for_s: float, side_min_m: float, rear_blocked: bool,
                 trail_len_m: float, since_last_s: float,
                 blocked_s: float = 1.0, side_tight_m: float = 0.5,
                 min_trail_m: float = 0.5, cooldown_s: float = 5.0) -> tuple[bool, str]:
    """是否该开始后退让位。返回 (是否触发, 原因)。"""
    if rear_blocked:
        return False, 'rear-blocked'
    if side_min_m >= side_tight_m:
        return False, 'side-clear'
    if blocked_for_s < blocked_s:
        return False, 'not-blocked-long-enough'
    if trail_len_m < min_trail_m:
        return False, 'trail-too-short'
    if since_last_s < cooldown_s:
        return False, 'cooldown'
    return True, 'start'


def should_stop(*, side_min_m: float, rear_blocked: bool, reversed_m: float,
                elapsed_s: float, turn_clear_m: float = 0.9,
                max_reverse_m: float = 1.2, max_s: float = 10.0) -> tuple[bool, str]:
    """是否该结束后退。返回 (是否结束, 原因)。"""
    if rear_blocked:
        return True, 'rear-blocked'
    if side_min_m >= turn_clear_m:
        return True, 'enough-room'          # ← 主要结束条件：左右都够转弯了
    if reversed_m >= max_reverse_m:
        return True, 'max-distance'
    if elapsed_s >= max_s:
        return True, 'timeout'
    return False, 'reversing'
