"""火山引擎（豆包）语音合成的纯逻辑测试：不联网，传输层全部打桩。"""

import base64
import io
import json
import wave

import numpy as np
import pytest

from fwr_voice import tts as tts_module
from fwr_voice.tts import VolcengineTts, decode_tts_audio


@pytest.fixture(autouse=True)
def credentials(monkeypatch):
    monkeypatch.setenv("FWR_VOLC_APPID", "app-123")
    monkeypatch.setenv("FWR_VOLC_TOKEN", "tok-456")


def pcm_b64(samples=1600, value=1000):
    data = np.full(samples, value, dtype="<i2").tobytes()
    return base64.b64encode(data).decode()


def wav_b64(samples=800, rate=16000):
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(rate)
        handle.writeframes(np.full(samples, 500, dtype="<i2").tobytes())
    return base64.b64encode(buffer.getvalue()).decode()


class FakeTransport:
    def __init__(self, response):
        self.response = response
        self.calls = []

    def __call__(self, url, headers, payload):
        self.calls.append((url, headers, payload))
        return self.response


def make(transport, **kwargs):
    defaults = dict(voice_type="BV700_streaming", appid_env="FWR_VOLC_APPID",
                    token_env="FWR_VOLC_TOKEN", transport=transport)
    defaults.update(kwargs)
    return VolcengineTts(**defaults)


def test_payload_carries_credentials_and_text():
    voice = make(FakeTransport((200, b"{}")))
    payload = voice.payload("你好")
    assert payload["app"] == {"appid": "app-123", "token": "tok-456",
                              "cluster": "volcano_tts"}
    assert payload["audio"]["voice_type"] == "BV700_streaming"
    assert payload["audio"]["encoding"] == "pcm"
    assert payload["request"]["text"] == "你好"
    assert payload["request"]["text_type"] == "plain"
    assert payload["request"]["operation"] == "query"
    assert payload["request"]["reqid"]


def test_payload_rejects_missing_credentials(monkeypatch):
    monkeypatch.delenv("FWR_VOLC_TOKEN", raising=False)
    voice = make(FakeTransport((200, b"{}")))
    with pytest.raises(RuntimeError, match="appid/token"):
        voice.payload("你好")


def test_authorization_uses_semicolon_form():
    voice = make(FakeTransport((200, b"{}")))
    assert voice.headers()["Authorization"] == "Bearer;tok-456"


def test_speak_plays_decoded_audio(monkeypatch):
    played = {}
    monkeypatch.setattr(tts_module, "play_with_alsa",
                        lambda samples, rate, interrupt, device=None:
                        played.update(samples=samples, rate=rate, device=device))
    transport = FakeTransport(
        (200, json.dumps({"code": 3000, "data": pcm_b64(2400)}).encode()))
    voice = make(transport, sample_rate=24000, device="plughw:2,0")
    voice.speak("你好")
    assert transport.calls[0][0] == VolcengineTts.DEFAULT_ENDPOINT
    assert played["rate"] == 24000
    assert played["samples"].size == 2400
    assert played["device"] == "plughw:2,0"
    assert voice.last_audio_started is True


def test_speak_uses_rate_from_wav_header(monkeypatch):
    played = {}
    monkeypatch.setattr(tts_module, "play_with_alsa",
                        lambda samples, rate, interrupt, device=None:
                        played.update(samples=samples, rate=rate))
    transport = FakeTransport(
        (200, json.dumps({"code": 3000, "data": wav_b64(rate=16000)}).encode()))
    voice = make(transport, encoding="wav", sample_rate=24000)
    voice.speak("你好")
    assert played["rate"] == 16000
    assert played["samples"].size == 800


def test_speak_rejects_http_error():
    voice = make(FakeTransport((403, b'{"message":"bad token"}')))
    with pytest.raises(RuntimeError, match="HTTP 403"):
        voice.speak("你好")
    assert voice.last_audio_started is False


def test_speak_rejects_error_code():
    body = json.dumps({"code": 3001, "message": "voice not found"}).encode()
    voice = make(FakeTransport((200, body)))
    with pytest.raises(RuntimeError, match="3001"):
        voice.speak("你好")


def test_speak_rejects_empty_audio():
    voice = make(FakeTransport((200, json.dumps({"code": 3000, "data": ""}).encode())))
    with pytest.raises(RuntimeError, match="no audio"):
        voice.speak("你好")


def test_speak_rejects_non_json():
    voice = make(FakeTransport((200, b"<html>oops</html>")))
    with pytest.raises(RuntimeError, match="non-JSON"):
        voice.speak("你好")


def test_speak_rejects_blank_text():
    voice = make(FakeTransport((200, b"{}")))
    with pytest.raises(ValueError, match="empty text"):
        voice.speak("   ")


def test_constructor_validation():
    with pytest.raises(ValueError, match="voice_type"):
        VolcengineTts("")
    with pytest.raises(ValueError, match="speed"):
        VolcengineTts("x", speed=5.0)
    with pytest.raises(ValueError, match="HTTPS"):
        VolcengineTts("x", endpoint="http://example.com/tts")


def test_decode_pcm_handles_odd_byte_count():
    samples, rate = decode_tts_audio(b"\x01\x02\x03", "pcm", 24000)
    assert rate == 24000
    assert samples.size == 1


def test_stop_sets_interrupt():
    voice = make(FakeTransport((200, b"{}")))
    voice.stop()
    assert voice.interrupt.is_set()
