import pytest

from fwr_voice.contracts import TaskRequest
from fwr_voice.dialogue import DialogueResult, parse_dialogue, route_cloud
from fwr_voice.gate import GateState
from fwr_voice.providers import OpenAICompatibleProvider


def test_parse_answer_and_task():
    assert parse_dialogue('{"reply":"你好","task":null}', "r").reply == "你好"
    result = parse_dialogue('{"reply":"去大厅","task":{"task_type":"GO_TO_NAMED_LOCATION","target_id":"lobby"}}', "r")
    assert result.task == TaskRequest("r", "GO_TO_NAMED_LOCATION", "lobby")


@pytest.mark.parametrize("payload", [
    "not json", '{"reply":"x","task":null,"cmd_vel":1}',
    '{"reply":"x","task":{"task_type":"GO_TO_NAMED_LOCATION","target_id":"lobby","speed":1}}',
    '{"reply":"x","task":{"task_type":"GO_TO_NAMED_LOCATION","target_id":"/cmd_vel"}}',
])
def test_reject_untrusted_shapes(payload):
    with pytest.raises(ValueError):
        parse_dialogue(payload, "r")


def test_late_or_unknown_task_cannot_execute():
    state = GateState(False, True, True)
    result = DialogueResult("好的", TaskRequest("old", "GO_TO_NAMED_LOCATION", "lobby"), 1, 1, "fake")
    assert route_cloud(result, "new", state, {"lobby"}).kind == "clarify"
    result = DialogueResult("好的", TaskRequest("new", "GO_TO_NAMED_LOCATION", "missing"), 1, 1, "fake")
    assert route_cloud(result, "new", state, {"lobby"}).kind == "clarify"


def test_provider_uses_env_key_and_times_out(monkeypatch):
    monkeypatch.setenv("FWR_TEST_LLM_KEY", "secret")
    seen = {}

    class Response:
        def raise_for_status(self):
            return None

        def json(self):
            return {"choices": [{"message": {"content": '{"reply":"你好","task":null}'}}],
                    "usage": {"prompt_tokens": 10, "completion_tokens": 3}}

    def post(url, **kwargs):
        seen.update(kwargs)
        return Response()

    provider = OpenAICompatibleProvider("https://example.test/v1", "small", "FWR_TEST_LLM_KEY", post=post)
    result = provider.respond("你好", ("STOP",), "空闲", "r")
    assert result.reply == "你好"
    assert seen["timeout"] == 3.0
    assert seen["headers"]["Authorization"] == "Bearer secret"
