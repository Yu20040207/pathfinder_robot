"""Safety boundary for high-level tasks."""

from dataclasses import dataclass

from .contracts import TASK_TYPES, TaskDecision, TaskRequest
from .jog import parse_motion_target


@dataclass(frozen=True)
class GateState:
    busy: bool
    executor_ready: bool
    simulated: bool


def authorize(request: TaskRequest, state: GateState,
              allowed_location_ids: set[str]) -> TaskDecision:
    if request.task_type not in TASK_TYPES:
        return TaskDecision("clarify", None, "不支持该任务。")
    if request.task_type == "GO_TO_NAMED_LOCATION":
        if request.target_id not in allowed_location_ids:
            return TaskDecision("clarify", None, "地点不在允许列表中。")
        if state.busy:
            return TaskDecision("clarify", None, "机器人正忙，请先停止当前任务。")
    elif request.task_type == "SIM_JOG":
        if not state.simulated:
            return TaskDecision("clarify", None, "该短动作只允许在模拟模式使用。")
        try:
            parse_motion_target(request.target_id)
        except ValueError:
            return TaskDecision("clarify", None, "不支持该运动方向。")
    elif request.target_id:
        return TaskDecision("clarify", None, "该任务不能指定地点。")
    if not state.executor_ready and request.task_type != "STOP":
        return TaskDecision("clarify", None, "任务执行器未就绪。")
    replies = {"STOP": "收到", "QUERY_STATUS": "正在查询状态。",
               "GO_TO_NAMED_LOCATION": "当前为模拟模式，正在模拟前往。" if state.simulated else "正在前往。",
               "SIM_JOG": "收到"}
    return TaskDecision("task", request, replies[request.task_type])
