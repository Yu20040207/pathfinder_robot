"""Explicitly armed exploration transitions; cancellation is a hard barrier."""
from dataclasses import dataclass
import math


# AMCL 正常重定位/纠正可达 0.3 m；阈值太紧会把正常修正当成 kidnap。
def map_frame_jumped(old_map, old_odom, new_map, new_odom, threshold_m=.60):
    if old_map is None or old_odom is None:
        return False
    return math.dist((new_map[0] - old_map[0], new_map[1] - old_map[1]),
                     (new_odom[0] - old_odom[0], new_odom[1] - old_odom[1])) > threshold_m


def trial_duration_expired(started_at, now, duration_s, fixed_goal):
    return (not fixed_goal and started_at is not None and
            now - started_at > duration_s)


@dataclass(frozen=True)
class ExplorationDecision:
    action: str
    state: str


class ExplorationStateMachine:
    def __init__(self):
        self.enabled = False
        self.blocked = set()
        self._failures = {}
        self._consecutive_failures = 0
        self._pending_recovery = False
        self._canceling = False
        self._stopping = False
        self.fixed_goal = False
        self._fixed_dispatched = 0
        self._fixed_legs = 1

    @property
    def fixed_leg_index(self):
        return max(0, self._fixed_dispatched - 1)

    def start(self):
        self.fixed_goal = False
        self.enabled = True
        self._stopping = False

    def start_fixed(self, legs=1):
        if legs < 1:
            raise ValueError('fixed task needs at least one leg')
        self.fixed_goal = True
        self._fixed_dispatched = 0
        self._fixed_legs = legs
        self.enabled = True
        self._stopping = False

    def stop(self):
        self.enabled = False
        self._stopping = True

    def recovery_finished(self, success):
        self._pending_recovery = False
        if success:
            self._consecutive_failures = 0
        else:
            self._consecutive_failures += 1

    def update(self, map_ok, pose_ok, fault, goal_status, now_s, goal_index=None):
        if not map_ok or not pose_ok or fault:
            self.stop()
        if self._stopping:
            if goal_status in ('active', 'sending') and not self._canceling:
                self._canceling = True
                return ExplorationDecision('cancel', 'stopping')
            if goal_status in ('sending', 'canceling', 'canceling_pending', 'active'):
                return ExplorationDecision('wait', 'stopping')
            self._canceling = False
            self._stopping = False
            return ExplorationDecision('stop', 'stopped')
        if not self.enabled:
            return ExplorationDecision('stop', 'stopped')
        if self.fixed_goal:
            if goal_status in ('failed', 'canceled'):
                self.stop()
                return ExplorationDecision('stop', 'stopped')
            if goal_status in ('active', 'sending', 'canceling', 'canceling_pending'):
                return ExplorationDecision('wait', 'navigating')
            if goal_status == 'succeeded' and (
                self._fixed_dispatched == 0 or self._fixed_dispatched >= self._fixed_legs
            ):
                self.stop()
                return ExplorationDecision('stop', 'stopped')
            if self._fixed_dispatched >= self._fixed_legs:
                self.stop()
                return ExplorationDecision('stop', 'stopped')
            self._fixed_dispatched += 1
            return ExplorationDecision('dispatch', 'selecting')
        if goal_status == 'failed' and not self._canceling:
            self._canceling = True
            if goal_index is not None:
                self._failures[goal_index] = self._failures.get(goal_index, 0) + 1
                if self._failures[goal_index] >= 2:
                    self.blocked.add(goal_index)
            self._pending_recovery = True
            return ExplorationDecision('cancel', 'canceling')
        if self._canceling:
            if goal_status != 'canceled':
                return ExplorationDecision('wait', 'canceling')
            self._canceling = False
            if self._pending_recovery:
                return ExplorationDecision('recover', 'recovering')
        if self._consecutive_failures >= 3:
            self.stop()
            return ExplorationDecision('stop', 'stopped')
        if goal_status in ('active', 'sending', 'canceling', 'canceling_pending'):
            return ExplorationDecision('wait', 'navigating')
        if goal_status == 'succeeded':
            self._consecutive_failures = 0
        return ExplorationDecision('dispatch', 'selecting')
