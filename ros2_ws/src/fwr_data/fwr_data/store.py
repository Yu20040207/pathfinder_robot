"""SQLite persistence for enrollment and attendance; no network dependency."""

from array import array
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import math
import os
import re
import sqlite3
from typing import Sequence


PERSON_ID = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


@dataclass(frozen=True)
class EnrollmentSample:
    image_jpeg: bytes
    embedding: tuple[float, ...]


@dataclass(frozen=True)
class Template:
    person_id: str
    display_name: str
    embedding: tuple[float, ...]


@dataclass(frozen=True)
class AttendanceRecord:
    id: int
    person_id: str
    display_name: str
    occurred_at: str
    score: float
    device_id: str


class AttendanceStore:
    def __init__(self, path: Path):
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        descriptor = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            os.fchmod(descriptor, 0o600)
        finally:
            os.close(descriptor)
        with self._connect() as db:
            db.executescript("""
                CREATE TABLE IF NOT EXISTS people (
                    person_id TEXT PRIMARY KEY, display_name TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS samples (
                    id INTEGER PRIMARY KEY, person_id TEXT NOT NULL REFERENCES people(person_id) ON DELETE CASCADE,
                    image_jpeg BLOB NOT NULL, embedding BLOB NOT NULL
                );
                CREATE TABLE IF NOT EXISTS events (
                    id INTEGER PRIMARY KEY, person_id TEXT NOT NULL, display_name TEXT NOT NULL,
                    occurred_at TEXT NOT NULL, score REAL NOT NULL, device_id TEXT NOT NULL
                );
            """)

    def _connect(self):
        db = sqlite3.connect(self.path, timeout=5)
        db.execute("PRAGMA foreign_keys=ON")
        db.execute("PRAGMA journal_mode=WAL")
        return db

    def enroll(self, person_id: str, display_name: str, samples: Sequence[EnrollmentSample]) -> None:
        name = display_name.strip()
        if not PERSON_ID.fullmatch(person_id) or not 1 <= len(name) <= 80:
            raise ValueError("Invalid person ID or display name")
        if not 1 <= len(samples) <= 5:
            raise ValueError("Use 1 to 5 enrollment images")
        dimension = len(samples[0].embedding)
        if dimension == 0 or any(not s.image_jpeg or len(s.embedding) != dimension or
                                 any(not math.isfinite(value) for value in s.embedding) for s in samples):
            raise ValueError("Invalid enrollment sample")
        try:
            with self._connect() as db:
                db.execute("INSERT INTO people VALUES (?, ?)", (person_id, name))
                db.executemany("INSERT INTO samples(person_id,image_jpeg,embedding) VALUES (?,?,?)", [
                    (person_id, s.image_jpeg, array("f", s.embedding).tobytes()) for s in samples
                ])
        except sqlite3.IntegrityError as error:
            raise ValueError("Person ID already exists") from error

    def templates(self) -> list[Template]:
        with self._connect() as db:
            rows = db.execute("SELECT p.person_id,p.display_name,s.embedding FROM samples s JOIN people p ON p.person_id=s.person_id ORDER BY p.person_id,s.id").fetchall()
        return [Template(row[0], row[1], tuple(array("f", row[2]))) for row in rows]

    def people(self) -> list[tuple[str, str, int]]:
        with self._connect() as db:
            return db.execute("SELECT p.person_id,p.display_name,COUNT(s.id) FROM people p LEFT JOIN samples s ON s.person_id=p.person_id GROUP BY p.person_id ORDER BY p.person_id").fetchall()

    def delete_person(self, person_id: str) -> bool:
        with self._connect() as db:
            return db.execute("DELETE FROM people WHERE person_id=?", (person_id,)).rowcount > 0

    def record_event(self, person_id: str, display_name: str, occurred_at: datetime, score: float, device_id: str) -> int:
        return self.record_events_batch([(person_id, display_name, occurred_at, score, device_id)])[0]

    def record_events_batch(self, records: Sequence[tuple[str, str, datetime, float, str]]) -> list[int]:
        for _, _, occurred_at, score, _ in records:
            if occurred_at.tzinfo is None or not math.isfinite(score):
                raise ValueError("Timestamp must include a timezone and score must be finite")
        ids = []
        with self._connect() as db:
            for person_id, display_name, occurred_at, score, device_id in records:
                stamp = occurred_at.astimezone(timezone.utc).isoformat()
                cursor = db.execute("INSERT INTO events(person_id,display_name,occurred_at,score,device_id) VALUES (?,?,?,?,?)", (person_id, display_name, stamp, score, device_id))
                ids.append(cursor.lastrowid)
        return ids

    def events(self) -> list[AttendanceRecord]:
        with self._connect() as db:
            rows = db.execute("SELECT id,person_id,display_name,occurred_at,score,device_id FROM events ORDER BY id DESC").fetchall()
        return [AttendanceRecord(*row) for row in rows]
