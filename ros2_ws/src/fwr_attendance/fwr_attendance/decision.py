"""Temporal confirmation and per-person attendance cooldown."""

from collections import deque
from datetime import datetime, timedelta
from typing import Sequence

from fwr_vision.engine import FaceResult


class AttendanceDecision:
    def __init__(self, window_seconds: float = 2.0, hits_required: int = 3,
                 cooldown_seconds: float = 60.0, minimum_score: float = 0.363):
        self.window = timedelta(seconds=window_seconds)
        self.hits_required = hits_required
        self.cooldown = timedelta(seconds=cooldown_seconds)
        self.minimum_score = minimum_score
        self.hits: dict[str, deque[datetime]] = {}
        self.last_seen: dict[str, datetime] = {}
        self.last_confirmation: dict[str, datetime] = {}

    def observe(self, frame_time: datetime, observations: Sequence[FaceResult],
                now: datetime) -> list[FaceResult]:
        age = (now - frame_time).total_seconds()
        if frame_time.tzinfo is None or now.tzinfo is None or age > 2.0 or age < -1.0:
            return []
        confirmed = []
        used_ids = set()
        for face in observations:
            pid = face.person_id
            if not face.recognized or not pid or face.score < self.minimum_score or pid in used_ids:
                continue
            used_ids.add(pid)
            if frame_time <= self.last_seen.get(pid, datetime.min.replace(tzinfo=frame_time.tzinfo)):
                continue
            self.last_seen[pid] = frame_time
            if frame_time - self.last_confirmation.get(pid, datetime.min.replace(tzinfo=frame_time.tzinfo)) < self.cooldown:
                continue
            hits = self.hits.setdefault(pid, deque())
            hits.append(frame_time)
            while hits and frame_time - hits[0] > self.window:
                hits.popleft()
            if len(hits) >= self.hits_required:
                confirmed.append(face)
                self.last_confirmation[pid] = frame_time
                hits.clear()
        return confirmed

    def rollback_confirmation(self, person_id: str):
        self.last_confirmation.pop(person_id, None)
