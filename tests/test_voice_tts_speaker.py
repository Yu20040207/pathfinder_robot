from pathlib import Path
import sys

import numpy as np
import pytest

from fwr_voice.tts import SherpaVitsTts


MODEL = (Path.home() / ".local/share/four_wheel_robot/voice_models/"
         "sherpa-onnx-vits-zh-ll")


def test_selected_speaker_reaches_synthesizer(monkeypatch):
    tts = SherpaVitsTts(MODEL, speaker_id=2)
    seen = []

    class Engine:
        def generate(self, text, config):
            seen.append(config.sid)
            return type("Audio", (), {"samples": np.zeros(10, dtype=np.float32),
                                      "sample_rate": 16000})()

    tts.engine = Engine()
    monkeypatch.setitem(sys.modules, "sounddevice", None)
    monkeypatch.setattr("fwr_voice.tts.play_with_alsa", lambda *args: None)
    tts.speak("你好")
    assert seen == [2]


def test_invalid_speaker_is_rejected():
    with pytest.raises(ValueError, match="speaker"):
        SherpaVitsTts(MODEL, speaker_id=5)


def test_configured_speech_speed_reaches_synthesizer(monkeypatch):
    tts = SherpaVitsTts(MODEL, speaker_id=2, speed=0.75)
    seen = []

    class Engine:
        def generate(self, text, config):
            seen.append(config.speed)
            return type("Audio", (), {"samples": np.zeros(10, dtype=np.float32),
                                      "sample_rate": 16000})()

    tts.engine = Engine()
    monkeypatch.setitem(sys.modules, "sounddevice", None)
    monkeypatch.setattr("fwr_voice.tts.play_with_alsa", lambda *args: None)
    tts.speak("你好")
    assert seen == [0.75]


def test_invalid_speech_speed_is_rejected():
    with pytest.raises(ValueError, match="speed"):
        SherpaVitsTts(MODEL, speed=0)
