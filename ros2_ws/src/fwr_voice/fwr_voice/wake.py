"""Local wake session and recent TTS echo suppression."""

import re
import threading
import time

from .intent import normalize, parse_local


def is_goodbye(text: str) -> bool:
    phrase = normalize(text)
    return bool(re.fullmatch(
        r"(?:你)?(?:先)?(?:退下|退一下)(?:吧|啦)?|再见|拜拜(?:啦|了)?|结束对话|下次见|晚安",
        phrase))


class WakeGate:
    def __init__(self, phrase: str = "小明同学", timeout_s: float = 30.0):
        if not normalize(phrase) or timeout_s <= 0:
            raise ValueError("wake phrase and timeout must be set")
        self.phrase = phrase
        self.timeout_s = timeout_s
        self.active_until = 0.0
        self._lock = threading.Lock()
        self._prefix = re.compile(r"^\s*" + re.escape(phrase) + r"[\s，。！？、,.!?]*")

    def consume(self, text: str, now: float | None = None) -> str | None:
        """Return speech during the active session, empty string on wake-only, or None."""
        now = time.monotonic() if now is None else now
        with self._lock:
            local = parse_local(text, {})
            if local.request is not None and local.request.task_type == "STOP":
                if now < self.active_until:
                    self.active_until = now + self.timeout_s
                return text
            match = self._prefix.match(text)
            if match:
                command = text[match.end():].strip()
                self.active_until = now + self.timeout_s
                return command
            if normalize(text) in {"", "嗯", "嗯嗯", "呃", "啊", "哦", "唔"}:
                return None
            if now < self.active_until:
                self.active_until = now + self.timeout_s
                return text
            return None

    def touch(self, now: float | None = None) -> bool:
        """Restart the idle timeout after an answer while the session is active."""
        now = time.monotonic() if now is None else now
        with self._lock:
            if not self.active_until:
                return False
            self.active_until = now + self.timeout_s
            return True

    def end(self) -> bool:
        """End the current session after an explicit goodbye."""
        with self._lock:
            was_active = bool(self.active_until)
            self.active_until = 0.0
            return was_active

    def expire(self, now: float | None = None) -> bool:
        """End an elapsed session once; caller may announce the transition."""
        now = time.monotonic() if now is None else now
        with self._lock:
            if self.active_until and now >= self.active_until:
                self.active_until = 0.0
                return True
            return False


class EchoGuard:
    def __init__(self, hold_s: float = 3.0):
        self.hold_s = hold_s
        self._recent = []
        self._lock = threading.Lock()

    def remember(self, text: str, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        with self._lock:
            self._recent = [(phrase, until) for phrase, until in self._recent if now < until]
            self._recent.append((normalize(text), now + self.hold_s))

    def is_echo(self, text: str, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        phrase = normalize(text)
        with self._lock:
            self._recent = [(spoken, until) for spoken, until in self._recent if now < until]
            return bool(phrase) and any(phrase == spoken for spoken, _ in self._recent)
