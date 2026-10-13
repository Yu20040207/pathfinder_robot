import struct
import sys
import threading
from types import SimpleNamespace
import wave

import pytest

from fwr_voice.audio import AudioClip, AlsaMicrophoneInput, MicrophoneInput, WavFileInput, capture_until_silence
from fwr_voice.asr import SherpaSenseVoice
from fwr_voice.tts import CallbackTts, SherpaVitsTts, play_with_alsa


def test_wav_file_input_requires_mono_16k_pcm(tmp_path):
    path = tmp_path / "ok.wav"
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(16000)
        wav.writeframes(struct.pack("<100h", *([1] * 100)))
    clip = WavFileInput(path).record_once(3)
    assert clip.sample_rate_hz == 16000
    assert len(clip.pcm16) == 200
    with pytest.raises(ValueError):
        WavFileInput(path).record_once(0.001)


def test_capture_stops_after_silence_and_caps_length():
    loud = struct.pack("<320h", *([5000] * 320))
    silent = bytes(len(loud))
    clip = capture_until_silence(iter([silent, loud, loud, silent, silent, silent]),
                                 16000, max_duration_s=1, threshold=0.01,
                                 trailing_silence_s=0.06)
    assert clip.pcm16.startswith(silent + loud)
    assert len(clip.pcm16) == 6 * len(loud)


def test_capture_keeps_only_recent_audio_before_speech():
    loud = struct.pack("<320h", *([5000] * 320))
    silent = bytes(len(loud))
    clip = capture_until_silence(iter([silent] * 15 + [loud, silent, silent, silent]),
                                 16000, max_duration_s=2, threshold=0.01,
                                 trailing_silence_s=0.06, pre_roll_s=0.2)
    assert clip.pcm16.startswith(silent * 10 + loud)
    assert len(clip.pcm16) == 14 * len(loud)


def test_capture_can_be_stopped_between_blocks():
    import threading
    stopped = threading.Event()
    loud = struct.pack("<320h", *([5000] * 320))
    def blocks():
        yield loud
        stopped.set()
        yield loud
    clip = capture_until_silence(blocks(), 16000, 1, stop_event=stopped)
    assert len(clip.pcm16) == len(loud)


def test_asr_empty_result_and_tts_error():
    class Stream:
        result = type("Result", (), {"text": ""})()

        def accept_waveform(self, rate, samples):
            assert rate == 16000

    class Recognizer:
        def create_stream(self):
            return Stream()

        def decode_stream(self, stream):
            return None

    asr = SherpaSenseVoice(recognizer=Recognizer())
    assert asr.transcribe(AudioClip(bytes(320), 16000, 0)).text == ""
    with pytest.raises(RuntimeError):
        CallbackTts(lambda text: (_ for _ in ()).throw(RuntimeError("speaker lost"))).speak("你好")


def test_alsa_capture_terminates_recorder():
    import io
    loud = struct.pack("<320h", *([5000] * 320))
    class Process:
        def __init__(self):
            self.stdout = io.BytesIO(loud + bytes(len(loud)) * 3)
            self.terminated = False

        def terminate(self):
            self.terminated = True

        def wait(self, timeout=None):
            return 0

    proc = Process()
    mic = AlsaMicrophoneInput(popen=lambda *args, **kwargs: proc)
    clip = mic.record_once(0.08)
    assert clip.pcm16.startswith(loud)
    assert proc.terminated


def test_alsa_playback_writes_pcm16():
    import io
    import threading
    import numpy as np
    class Sink(io.BytesIO):
        def close(self):
            pass
    class Process:
        stdin = Sink()

        def wait(self, timeout=None):
            return 0

    proc = Process()
    play_with_alsa(np.array([0.0, 0.5], dtype=np.float32), 16000,
                   threading.Event(), popen=lambda *args, **kwargs: proc)
    assert len(proc.stdin.getvalue()) == 4


def test_alsa_playback_stop_is_not_error():
    import threading
    import numpy as np
    class Sink:
        def close(self):
            raise BrokenPipeError()

        def write(self, data):
            raise AssertionError("stopped playback must not write")
    class Process:
        stdin = Sink()
        terminated = False

        def terminate(self):
            self.terminated = True

        def wait(self, timeout=None):
            return -15
    proc = Process()
    stopped = threading.Event()
    stopped.set()
    play_with_alsa(np.zeros(100, dtype=np.float32), 16000, stopped,
                   popen=lambda *args, **kwargs: proc)
    assert proc.terminated


def test_incomplete_sounddevice_uses_alsa(monkeypatch):
    from fwr_voice import audio, tts
    import numpy as np

    monkeypatch.setitem(sys.modules, "sounddevice", SimpleNamespace())
    captured = []
    monkeypatch.setattr(audio.AlsaMicrophoneInput, "record_once",
                        lambda self, duration, stop_event: captured.append((self.device, duration)) or AudioClip(b"", 16000, 0))
    MicrophoneInput(device="mic").record_once(2)
    assert captured == [("mic", 2)]

    monkeypatch.setitem(sys.modules, "sherpa_onnx", SimpleNamespace(GenerationConfig=SimpleNamespace))
    monkeypatch.setattr(tts, "play_with_alsa",
                        lambda samples, sample_rate, interrupt: captured.append((sample_rate, len(samples))))
    voice = object.__new__(SherpaVitsTts)
    voice.engine = SimpleNamespace(generate=lambda text, config: SimpleNamespace(samples=np.zeros(8), sample_rate=16000))
    voice.speaker_id = 0
    voice.speed = 1.0
    voice.interrupt = threading.Event()
    voice.playback_lock = threading.Lock()
    voice.speak("测试")
    assert captured[-1] == (16000, 8)
