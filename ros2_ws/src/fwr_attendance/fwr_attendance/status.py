"""Private, process-shared last error for the local operator page."""

import json
import os
from pathlib import Path
import tempfile


class StatusFile:
    def __init__(self, path: Path):
        self.path = Path(path)

    def set_error(self, message: str):
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        descriptor, name = tempfile.mkstemp(prefix="status-", dir=self.path.parent)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump({"last_error": str(message)[:500]}, stream, ensure_ascii=False)
            Path(name).replace(self.path)
        finally:
            Path(name).unlink(missing_ok=True)

    def get_error(self) -> str | None:
        try:
            return json.loads(self.path.read_text(encoding="utf-8")).get("last_error")
        except (FileNotFoundError, ValueError, OSError):
            return None

    def clear_error(self):
        self.path.unlink(missing_ok=True)
