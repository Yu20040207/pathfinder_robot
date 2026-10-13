import os
import threading
import time
from types import SimpleNamespace

from fwr_voice.tts import FallbackTts, DashscopeCosyVoiceTts


def test_cloud_voice_uses_selected_model_and_pcm_audio(monkeypatch):
    calls = []
    class Synth:
        def __init__(self, **kwargs):
            calls.append(kwargs)
        def call(self, text):
            calls.append(text)
            calls[0]["callback"].on_data(b"\x00\x00")
            calls[0]["callback"].on_complete()
    class Sink:
        def write(self, data):
            calls.append(data)
        def close(self):
            pass
    fake = SimpleNamespace(api_key=None, base_websocket_api_url=None)
    api = SimpleNamespace(SpeechSynthesizer=Synth, ResultCallback=object,
                          AudioFormat=SimpleNamespace(PCM_22050HZ_MONO_16BIT="pcm"))
    monkeypatch.setenv("FWR_TEST_TTS_KEY", "secret")
    tts = DashscopeCosyVoiceTts("longanya_v3", key_env="FWR_TEST_TTS_KEY",
                               dashscope_module=fake, tts_api=api,
                               output_factory=lambda: Sink())
    tts.speak("你好")
    assert calls[0]["model"] == "cosyvoice-v3-flash"
    assert calls[0]["voice"] == "longanya_v3"
    assert calls[0]["format"] == "pcm"
    assert calls[1:] == ["你好", b"\x00\x00"]


def test_cloud_failure_falls_back_to_local():
    spoken = []
    class Cloud:
        def speak(self, text):
            raise RuntimeError("offline")
    local = SimpleNamespace(speak=lambda text: spoken.append(text))
    FallbackTts(Cloud(), local).speak("你好")
    assert spoken == ["你好"]


def test_cloud_waits_for_async_audio_before_closing(monkeypatch):
    played = []
    class Synth:
        def __init__(self, **kwargs):
            self.callback = kwargs["callback"]
        def call(self, text):
            threading.Timer(0.03, lambda: (self.callback.on_data(b"audio"),
                                           self.callback.on_complete())).start()
    class Sink:
        def write(self, data):
            played.append(data)
        def close(self):
            played.append(b"closed")
    monkeypatch.setenv("FWR_TEST_TTS_KEY", "secret")
    api = SimpleNamespace(SpeechSynthesizer=Synth, ResultCallback=object,
                          AudioFormat=SimpleNamespace(PCM_22050HZ_MONO_16BIT="pcm"))
    tts = DashscopeCosyVoiceTts("longanya_v3", key_env="FWR_TEST_TTS_KEY",
                               dashscope_module=SimpleNamespace(), tts_api=api,
                               output_factory=Sink)
    tts.speak("你好")
    assert played == [b"audio", b"closed"]
