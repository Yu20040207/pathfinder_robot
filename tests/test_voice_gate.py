from fwr_voice.contracts import TaskRequest
from fwr_voice.gate import GateState, authorize


def test_gate_rejects_invalid_and_busy_motion():
    state = GateState(busy=True, executor_ready=True, simulated=True)
    assert authorize(TaskRequest("1", "GO_TO_NAMED_LOCATION", "lab"), state, {"lab"}).kind == "clarify"
    assert authorize(TaskRequest("1", "GO_TO_NAMED_LOCATION", "secret"), GateState(False, True, True), {"lab"}).kind == "clarify"
    assert authorize(TaskRequest("1", "MOVE_RAW"), GateState(False, True, True), {"lab"}).kind == "clarify"


def test_stop_can_cancel_while_busy_and_simulation_is_explicit():
    state = GateState(busy=True, executor_ready=True, simulated=True)
    assert authorize(TaskRequest("1", "STOP"), state, set()).kind == "task"
    decision = authorize(TaskRequest("2", "GO_TO_NAMED_LOCATION", "lab"), GateState(False, True, True), {"lab"})
    assert decision.kind == "task"
    assert "模拟" in decision.reply
    assert authorize(TaskRequest("2", "GO_TO_NAMED_LOCATION", "lab"), GateState(False, False, True), {"lab"}).kind == "clarify"
    assert authorize(TaskRequest("3", "STOP"), GateState(False, False, True), set()).kind == "task"


def test_new_motion_is_allowed_while_another_task_runs():
    decision = authorize(TaskRequest("4", "SIM_JOG", "right"),
                         GateState(True, True, True), set())
    assert decision.kind == "task"
    assert decision.reply == "收到"
