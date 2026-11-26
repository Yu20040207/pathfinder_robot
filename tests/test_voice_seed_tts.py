"""豆包语音 Seed-TTS（语音合成 2.0）的纯逻辑测试：不联网，传输层全部打桩。

协议的来由见 `tts.SeedTts` 文档：这一套和经典版 `VolcengineTts` 完全是两回事
（`X-Api-Key` + `X-Api-Resource-Id`，响应是 NDJSON 流），所以单测必须钉住
"别又退回经典版写法"。
"""

import base64
import json

import numpy as np
import pytest

from fwr_voice import tts as tts_module
from fwr_voice.tts import SeedTts


@pytest.fixture(autouse=True)
def credentials(monkeypatch):
    monkeypatch.setenv("FWR_VOLC_APIKEY", "9e7dac36-aaaa-bbbb-cccc-1234562cd915")


def pcm_b64(samples=800, value=1200):
    return base64.b64encode(np.full(samples, value, dtype="<i2").tobytes()).decode()


def ndjson(*chunks):
    """把若干 dict 拼成 NDJSON 响应体（每块一行，跟真实服务一致）。"""
    return ("\n".join(json.dumps(c, ensure_ascii=False) for c in chunks)).encode()


class FakeTransport:
    def __init__(self, status=200, body=b""):
        self.status, self.body = status, body
        self.calls = []

    def __call__(self, url, headers, payload, **_kwargs):
        self.calls.append({"url": url, "headers": headers, "payload": payload})
        return self.status, self.body


def play_recorder(monkeypatch):
    played = []

    def fake_play(samples, rate, interrupt, **_kwargs):
        played.append((samples.copy(), rate))

    monkeypatch.setattr(tts_module, "play_with_alsa", fake_play)
    return played


def make(transport, **kwargs):
    params = dict(apikey_env="FWR_VOLC_APIKEY", transport=transport)
    params.update(kwargs)
    return SeedTts("saturn_zh_female_keainvsheng_tob", **params)


# --- 请求形状 -------------------------------------------------------------

def test_payload_matches_seed_v3_shape():
    payload = make(FakeTransport(200, b"")).payload("你好")
    assert payload["user"]["uid"]
    assert payload["req_params"]["text"] == "你好"
    assert payload["req_params"]["speaker"] == "saturn_zh_female_keainvsheng_tob"
    assert payload["req_params"]["audio_params"] == {"format": "pcm", "sample_rate": 24000}
    # 经典版的 app/cluster 字段在这一套里不该出现
    assert "app" not in payload
    assert "audio" not in payload


def test_headers_use_api_key_and_resource_id():
    transport = FakeTransport(200, ndjson({"code": 0, "data": pcm_b64()}))
    headers = make(transport).headers()
    assert headers["X-Api-Key"] == "9e7dac36-aaaa-bbbb-cccc-1234562cd915"
    assert headers["X-Api-Resource-Id"] == "seed-tts-2.0"
    # 千万别再走 `Bearer;` 那套：那是经典版的，会被判 invalid auth token
    assert "Authorization" not in headers


def test_custom_resource_id_is_forwarded():
    transport = FakeTransport(200, ndjson({"code": 0, "data": pcm_b64()}))
    headers = make(transport, resource_id="seed-tts-3.0").headers()
    assert headers["X-Api-Resource-Id"] == "seed-tts-3.0"


def test_missing_api_key_raises(monkeypatch):
    monkeypatch.delenv("FWR_VOLC_APIKEY", raising=False)
    with pytest.raises(RuntimeError, match="api key unavailable"):
        make(FakeTransport(200, b"")).headers()


def test_api_key_is_stripped(monkeypatch):
    monkeypatch.setenv("FWR_VOLC_APIKEY", "  abc-123  \n")
    transport = FakeTransport(200, ndjson({"code": 0, "data": pcm_b64()}))
    assert make(transport).headers()["X-Api-Key"] == "abc-123"


# --- 响应解析（NDJSON 流） -------------------------------------------------

def test_parse_stream_joins_chunks_and_ignores_end_code():
    body = ndjson({"code": 0, "data": pcm_b64(400)},
                  {"code": 0, "data": pcm_b64(400)},
                  {"code": SeedTts.END_CODE, "data": ""})
    raw, errors = SeedTts.parse_stream(body)
    assert errors == []
    assert len(raw) == 1600  # 400 + 400 个 int16
    assert raw[:2] == np.int16(1200).tobytes()


def test_parse_stream_reports_nonzero_codes():
    body = ndjson({"code": 55000000,
                   "message": "resource ID is mismatched with speaker related resource"})
    raw, errors = SeedTts.parse_stream(body)
    assert raw == b""
    assert len(errors) == 1
    assert "55000000" in errors[0]
    assert "mismatched" in errors[0]


def test_parse_stream_skips_blank_and_garbage_lines():
    body = b'\n{"code":0,"data":"' + pcm_b64(10).encode() + b'"}\nnot-json\n\n'
    raw, errors = SeedTts.parse_stream(body)
    assert errors == []
    assert len(raw) == 20


def test_parse_stream_handles_empty_body():
    assert SeedTts.parse_stream(b"") == (b"", [])


# --- speak 全链路 ---------------------------------------------------------

def test_speak_plays_concatenated_audio(monkeypatch):
    played = play_recorder(monkeypatch)
    body = ndjson({"code": 0, "data": pcm_b64(2400)},
                  {"code": 0, "data": pcm_b64(2400)},
                  {"code": SeedTts.END_CODE})
    tts = make(FakeTransport(200, body))
    tts.speak("你好")
    assert tts.last_audio_started is True
    assert len(played) == 1
    samples, rate = played[0]
    assert rate == 24000
    assert samples.size == 4800


def test_speak_reports_http_error():
    tts = make(FakeTransport(403, json.dumps(
        {"header": {"code": 45000030,
                    "message": "[resource_id=seed-tts-3.0] requested resource not granted"}}
    ).encode()))
    with pytest.raises(RuntimeError, match="HTTP 403"):
        tts.speak("你好")


def test_speak_raises_on_business_error_with_http_200():
    """真实踩过：HTTP 200 但 code 非 0（音色和资源不匹配），必须当失败处理。"""
    tts = make(FakeTransport(200, ndjson(
        {"code": 55000000, "message": "resource ID is mismatched with speaker related resource"})))
    with pytest.raises(RuntimeError, match="mismatched"):
        tts.speak("你好")


def test_speak_raises_when_no_audio():
    tts = make(FakeTransport(200, ndjson({"code": SeedTts.END_CODE})))
    with pytest.raises(RuntimeError, match="no audio"):
        tts.speak("你好")


def test_speak_does_not_set_audio_started_on_failure():
    tts = make(FakeTransport(401, b'{"code":45000010,"message":"load grant"}'))
    with pytest.raises(RuntimeError):
        tts.speak("你好")
    assert tts.last_audio_started is False


def test_speak_rejects_blank_text():
    with pytest.raises(ValueError, match="empty text"):
        make(FakeTransport(200, b"")).speak("   ")


def test_stop_sets_interrupt():
    tts = make(FakeTransport(200, b""))
    tts.stop()
    assert tts.interrupt.is_set()


# --- 构造校验 -------------------------------------------------------------

@pytest.mark.parametrize("kwargs, message", [
    ({"voice": ""}, "voice required"),
    ({"endpoint": "http://insecure"}, "HTTPS"),
    ({"sample_rate": 0}, "sample_rate"),
])
def test_constructor_validation(kwargs, message):
    voice = kwargs.pop("voice", "saturn_zh_female_keainvsheng_tob")
    with pytest.raises(ValueError, match=message):
        SeedTts(voice, transport=FakeTransport(200, b""), **kwargs)


def test_endpoint_default_is_v3_unidirectional():
    assert SeedTts.DEFAULT_ENDPOINT.endswith("/api/v3/tts/unidirectional")
    assert SeedTts.DEFAULT_RESOURCE_ID == "seed-tts-2.0"
