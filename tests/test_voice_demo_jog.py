from fwr_voice.contracts import TaskRequest
from fwr_voice.dialogue import parse_dialogue
from fwr_voice.gate import GateState, authorize
from fwr_voice.intent import parse_local
from fwr_voice.jog import run_jog, run_motion


def test_local_jog_has_only_four_fixed_directions():
    locations = {"lobby": ("大厅",)}
    for phrase, direction in (("前进一下", "forward:0.12"), ("后退", "backward"),
                              ("左转一下", "left:0.35"), ("右转", "right")):
        request = parse_local(phrase, locations).request
        assert (request.task_type, request.target_id) == ("SIM_JOG", direction)
    assert parse_local("前进两米", locations).request.target_id == "forward:2"


def test_continuous_and_distance_motion_are_local():
    locations = {}
    assert parse_local("前进", locations).request.target_id == "forward"
    assert parse_local("前进两米", locations).request.target_id == "forward:2"
    assert parse_local("后退0.5米", locations).request.target_id == "backward:0.5"
    assert parse_local("前进两米", locations).reply == "收到"


def test_continuous_motion_runs_until_cancel_then_publishes_zero():
    messages = []
    assert run_motion("forward", lambda v, w: messages.append((v, w)),
                      lambda: len(messages) >= 5, lambda _: None) is True
    assert len(messages) == 6
    assert messages[-1] == (0, 0)


def test_distance_motion_ends_at_requested_distance():
    messages = []
    distance = [0.0]
    def sleep(dt):
        distance[0] += 0.12 * dt
    assert run_motion("forward:2", lambda v, w: messages.append((v, w)),
                      lambda: False, sleep, lambda: distance[0]) is False
    assert 1.99 <= distance[0] <= 2.02
    assert messages[-1] == (0, 0)


def test_jog_is_simulation_only_and_cloud_cannot_request_it():
    request = TaskRequest("r", "SIM_JOG", "forward")
    assert authorize(request, GateState(False, True, True), set()).kind == "task"
    assert authorize(request, GateState(False, True, False), set()).kind == "clarify"
    assert authorize(request, GateState(True, True, True), set()).kind == "task"
    assert authorize(TaskRequest("r", "SIM_JOG", "fast"),
                     GateState(False, True, True), set()).kind == "clarify"
    payload = '{"reply":"好","task":{"task_type":"SIM_JOG","target_id":"forward"}}'
    try:
        parse_dialogue(payload, "r")
    except ValueError:
        pass
    else:
        raise AssertionError("cloud must not issue simulated motion")


def test_jog_publishes_bounded_commands_and_final_zero():
    messages = []
    assert run_jog("forward", lambda v, w: messages.append((v, w)),
                   lambda: False, lambda _: None) is False
    assert 1 <= len(messages) <= 12
    assert all(0 < v <= 0.12 and w == 0 for v, w in messages[:-1])
    assert messages[-1] == (0, 0)


def test_jog_cancel_stops_with_zero():
    messages = []
    assert run_jog("left", lambda v, w: messages.append((v, w)),
                   lambda: len(messages) >= 2, lambda _: None) is True
    assert len(messages) == 3
    assert all(v == 0 and w > 0 for v, w in messages[:-1])
    assert messages[-1] == (0, 0)
