"""Local sherpa-onnx SenseVoice and Paraformer adapters."""

from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np

from .audio import AudioClip


@dataclass(frozen=True)
class Transcript:
    text: str
    provider: str
    latency_ms: float


class SherpaSenseVoice:
    def __init__(self, model_dir: str | Path | None = None, *, recognizer=None,
                 family: str = "sensevoice", num_threads: int = 2):
        if recognizer is None:
            import sherpa_onnx
            if model_dir is None:
                raise ValueError("model_dir required")
            path = Path(model_dir)
            tokens = path / "tokens.txt"
            model = path / "model.int8.onnx"
            if not model.is_file():
                model = path / "model.onnx"
            if not model.is_file() or not tokens.is_file():
                raise FileNotFoundError("ASR model or tokens missing")
            factory = (sherpa_onnx.OfflineRecognizer.from_sense_voice if family == "sensevoice"
                       else sherpa_onnx.OfflineRecognizer.from_paraformer)
            options = {"model": str(model), "tokens": str(tokens), "num_threads": num_threads}
            if family == "sensevoice":
                options.update(language="zh", use_itn=True)
            recognizer = factory(**options)
        self.recognizer = recognizer
        self.family = family

    def transcribe(self, clip: AudioClip) -> Transcript:
        if clip.sample_rate_hz != 16000 or len(clip.pcm16) % 2:
            raise ValueError("expected 16 kHz PCM16")
        start = time.monotonic()
        stream = self.recognizer.create_stream()
        samples = np.frombuffer(clip.pcm16, dtype="<i2").astype(np.float32) / 32768
        stream.accept_waveform(16000, samples)
        self.recognizer.decode_stream(stream)
        return Transcript(stream.result.text.strip(), self.family,
                          (time.monotonic() - start) * 1000)
