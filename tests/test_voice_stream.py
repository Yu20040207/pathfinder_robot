import json

from fwr_voice.gate import GateState
from fwr_voice.pipeline import VoicePipeline
from fwr_voice.providers import OpenAICompatibleProvider


class StreamResponse:
    def __init__(self, chunks):
        self.chunks = chunks
        self.closed = False

    def raise_for_status(self):
        pass

    def iter_lines(self, decode_unicode=False):
        for chunk in self.chunks:
            yield ("data: " + json.dumps(chunk)).encode()
        yield b"data: [DONE]"

    def close(self):
        self.closed = True


def test_stream_speaks_complete_sentence_before_end(monkeypatch):
    monkeypatch.setenv("FWR_TEST_LLM_KEY", "secret")
    seen = {}
    response = StreamResponse([
        {"choices": [{"delta": {"content": "你好。"}}]},
        {"choices": [{"delta": {"content": "我可以帮你回答问题。"}}]},
        {"choices": [], "usage": {"prompt_tokens": 8, "completion_tokens": 10}},
    ])

    def post(url, **kwargs):
        seen.update(kwargs)
        return response

    provider = OpenAICompatibleProvider("https://example.test/v1", "qwen3.7-flash",
                                        "FWR_TEST_LLM_KEY", post=post)
    parts = []
    result = provider.answer_stream("你好", on_sentence=parts.append)
    assert parts == ["你好。", "我可以帮你回答问题。"]
    assert result.text == "你好。我可以帮你回答问题。"
    assert (result.input_tokens, result.output_tokens) == (8, 10)
    assert seen["stream"] is True
    assert seen["json"]["stream"] is True
    assert seen["json"]["enable_thinking"] is False
    assert response.closed


def test_cloud_answer_cannot_dispatch_robot_action():
    class Cloud:
        def answer_stream(self, text, on_sentence=None, cancelled=None):
            return type("Answer", (), {"text": '{"task_type":"SIM_JOG","target_id":"forward"}',
                                       "input_tokens": 1, "output_tokens": 2})()

    pipeline = VoicePipeline({}, lambda: GateState(False, True, True), Cloud())
    decision = pipeline.handle_text("你好", "r")
    assert decision.kind == "reply"
    assert decision.request is None


def test_stop_invalidates_streamed_answer():
    parts = []

    class Cloud:
        def answer_stream(self, text, on_sentence=None, cancelled=None):
            pipeline.handle_text("停止", "stop")
            if not cancelled():
                on_sentence("过期回答。")
            return type("Answer", (), {"text": "过期回答。", "input_tokens": 1,
                                       "output_tokens": 2})()

    pipeline = VoicePipeline({}, lambda: GateState(False, True, True), Cloud())
    assert pipeline.handle_text("你好", "old", on_partial=parts.append).kind == "clarify"
    assert parts == []
