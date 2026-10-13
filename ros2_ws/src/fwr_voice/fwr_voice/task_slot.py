"""Thread-safe ownership of the active ROS task goal."""

import threading


class TaskSlot:
    def __init__(self):
        self._lock = threading.Lock()
        self._pending = False
        self._handle = None
        self._cancel_pending = False
        self._replacement = None

    def is_busy(self) -> bool:
        with self._lock:
            return self._pending or self._handle is not None

    def reserve(self) -> bool:
        with self._lock:
            if self._pending or self._handle is not None:
                return False
            self._pending = True
            return True

    def request_cancel(self):
        with self._lock:
            self._replacement = None
            if self._pending:
                self._cancel_pending = True
            return self._handle

    def reserve_or_replace(self, request):
        with self._lock:
            if self._pending or self._handle is not None:
                self._replacement = request
                if self._pending:
                    self._cancel_pending = True
                return False, self._handle
            self._pending = True
            return True, None

    def accept(self, handle) -> bool:
        with self._lock:
            self._pending = False
            self._handle = handle
            cancel = self._cancel_pending
            self._cancel_pending = False
            return cancel

    def finish(self):
        with self._lock:
            replacement = self._replacement
            self._replacement = None
            self._pending = replacement is not None
            self._handle = None
            self._cancel_pending = False
            return replacement
