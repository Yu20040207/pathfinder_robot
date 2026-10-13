"""Strict parsing and routing of untrusted model output."""

from dataclasses import dataclass
import json

from .contracts import CLOUD_TASK_TYPES, TaskDecision, TaskRequest
from .gate import GateState, authorize


@dataclass(frozen=True)
class DialogueResult:
    reply: str
    task: TaskRequest | None
    input_tokens: int
    output_tokens: int
    provider: str


def parse_dialogue(payload: str, request_id: str, *, input_tokens: int = 0,
                   output_tokens: int = 0, provider: str = "") -> DialogueResult:
    obj = json.loads(payload)
    if not isinstance(obj, dict) or set(obj) != {"reply", "task"} or not isinstance(obj["reply"], str):
        raise ValueError("invalid dialogue response")
    task = obj["task"]
    request = None
    if task is not None:
        if (not isinstance(task, dict) or set(task) != {"task_type", "target_id"}
                or not all(isinstance(value, str) for value in task.values())
                or task["task_type"] not in CLOUD_TASK_TYPES
                or "/" in task["target_id"]):
            raise ValueError("invalid task")
        request = TaskRequest(request_id, task["task_type"], task["target_id"])
    return DialogueResult(obj["reply"][:500], request, input_tokens, output_tokens, provider)


def route_cloud(result: DialogueResult, active_request_id: str, state: GateState,
                allowed_location_ids: set[str]) -> TaskDecision:
    if result.task is None:
        return TaskDecision("reply", None, result.reply)
    if result.task.request_id != active_request_id:
        return TaskDecision("clarify", None, "请求已过期。")
    return authorize(result.task, state, allowed_location_ids)
