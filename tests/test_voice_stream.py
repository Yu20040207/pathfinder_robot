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


def test_stream_decodes_utf8_bytes_not_latin1(monkeypatch):
    """方舟的流式响应不带 charset，requests 会按 latin-1 解 → 中文变乱码。

    回归：必须自己按 UTF-8 解字节，否则 `json.loads` 还会因为多字节字符被
    行切分劈开而报 "Unterminated string"。
    """
    monkeypatch.setenv("FWR_TEST_LLM_KEY", "secret")
    payload = json.dumps({"choices": [{"delta": {"content": "我是家庭服务机器人。"}}]},
                         ensure_ascii=False).encode("utf-8")

    class Utf8Response(StreamResponse):
        def iter_lines(self, decode_unicode=False):
            # 关键：吐 bytes（跟 requests 的 decode_unicode=False 一致）
            yield b"data: " + payload
            yield b"data: [DONE]"

    provider = OpenAICompatibleProvider("https://example.test/v1", "doubao",
                                        "FWR_TEST_LLM_KEY",
                                        post=lambda url, **kw: Utf8Response([]))
    result = provider.answer_stream("你好")
    assert result.text == "我是家庭服务机器人。"


def test_extra_payload_merged_into_request(monkeypatch):
    """方舟关思考要传 thinking={"type":"disabled"}；必须能透传进去。"""
    monkeypatch.setenv("FWR_TEST_LLM_KEY", "secret")
    seen = {}
    response = StreamResponse([
        {"choices": [{"delta": {"content": "好的。"}}]},
    ])

    def post(url, **kwargs):
        seen.update(kwargs)
        return response

    provider = OpenAICompatibleProvider(
        "https://example.test/v1", "doubao", "FWR_TEST_LLM_KEY", post=post,
        extra_payload={"thinking": {"type": "disabled"}})
    provider.answer_stream("你好")
    assert seen["json"]["thinking"] == {"type": "disabled"}
    # 老字段仍在，不影响通义那类服务商
    assert seen["json"]["enable_thinking"] is False


def test_stream_ignores_reasoning_content(monkeypatch):
    """思考型模型先吐 reasoning_content；不能把"思考过程"念出来。"""
    monkeypatch.setenv("FWR_TEST_LLM_KEY", "secret")
    response = StreamResponse([
        {"choices": [{"delta": {"reasoning_content": "用户问我问题。"}}]},
        {"choices": [{"delta": {"content": "你好。"}}]},
    ])
    provider = OpenAICompatibleProvider("https://example.test/v1", "doubao",
                                        "FWR_TEST_LLM_KEY",
                                        post=lambda url, **kw: response)
    result = provider.answer_stream("你好")
    assert result.text == "你好。"


def test_system_prompt_carries_current_datetime():
    """模型不知道今天几号会瞎编（实测把 2026-11-26 周四答成「星期三」）。

    必须把准确日期/星期/时间拼进 system prompt。
    """
    from datetime import datetime

    from fwr_voice.providers import build_system_prompt, time_sentence

    sentence = time_sentence(datetime(2026, 11, 26, 14, 30))
    assert "2026 年 11 月 26 日" in sentence
    assert "星期四" in sentence          # 2026-11-26 是星期四
    assert "14:30" in sentence

    prompt = build_system_prompt(datetime(2026, 11, 26, 9, 5))
    assert "中文语音助手" in prompt       # 原有角色设定没丢
    assert "2026 年 11 月 26 日" in prompt
    assert "09:05" in prompt


def test_answer_stream_sends_prompt_with_time(monkeypatch):
    """真正发出去的请求里必须带上时间，不能只在工具函数里拼。"""
    monkeypatch.setenv("FWR_TEST_LLM_KEY", "secret")
    seen = {}
    response = StreamResponse([{"choices": [{"delta": {"content": "好的。"}}]}])

    def post(url, **kwargs):
        seen.update(kwargs)
        return response

    provider = OpenAICompatibleProvider("https://example.test/v1", "doubao",
                                        "FWR_TEST_LLM_KEY", post=post)
    provider.answer_stream("现在几点")
    system = seen["json"]["messages"][0]["content"]
    assert seen["json"]["messages"][0]["role"] == "system"
    assert "现在是" in system and "星期" in system
