"""Model-independent voice decisions."""

from dataclasses import dataclass

TASK_TYPES = frozenset({"STOP", "QUERY_STATUS", "GO_TO_NAMED_LOCATION", "SIM_JOG"})
CLOUD_TASK_TYPES = TASK_TYPES - {"SIM_JOG"}


@dataclass(frozen=True)
class TaskRequest:
    request_id: str
    task_type: str
    target_id: str = ""


@dataclass(frozen=True)
class TaskDecision:
    kind: str
    request: TaskRequest | None
    reply: str
