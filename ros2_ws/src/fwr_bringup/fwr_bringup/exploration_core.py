"""Explicitly armed exploration transitions; cancellation is a hard barrier."""
from dataclasses import dataclass


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

    def start(self):
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
            if goal_status in ('active', 'failed', 'succeeded') and not self._canceling:
                self._canceling = True
                return ExplorationDecision('cancel', 'stopping')
            if goal_status == 'canceling':
                return ExplorationDecision('wait', 'stopping')
            self._canceling = False
            self._stopping = False
            return ExplorationDecision('stop', 'stopped')
        if not self.enabled:
            return ExplorationDecision('stop', 'stopped')
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
        if goal_status in ('active', 'sending', 'canceling'):
            return ExplorationDecision('wait', 'navigating')
        if goal_status == 'succeeded':
            self._consecutive_failures = 0
        return ExplorationDecision('dispatch', 'selecting')
