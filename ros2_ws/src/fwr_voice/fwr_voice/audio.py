"""16 kHz PCM capture and file injection."""

from dataclasses import dataclass
from pathlib import Path
import audioop
import subprocess
import time
import wave


@dataclass(frozen=True)
class AudioClip:
    pcm16: bytes
    sample_rate_hz: int
    ended_monotonic_s: float


class WavFileInput:
    def __init__(self, path: str | Path):
        self.path = Path(path)

    def record_once(self, max_duration_s: float) -> AudioClip:
        with wave.open(str(self.path), "rb") as wav:
            if (wav.getnchannels(), wav.getsampwidth(), wav.getframerate()) != (1, 2, 16000):
                raise ValueError("expected mono 16 kHz PCM16 WAV")
            if wav.getnframes() / 16000 > max_duration_s:
                raise ValueError("audio exceeds maximum duration")
            pcm = wav.readframes(wav.getnframes())
        return AudioClip(pcm, 16000, time.monotonic())


def capture_until_silence(chunks, sample_rate_hz: int, max_duration_s: float,
                          threshold: float = 0.02, trailing_silence_s: float = 0.5,
                          stop_event=None, pre_roll_s: float = 0.2) -> AudioClip:
    if sample_rate_hz <= 0 or max_duration_s <= 0 or pre_roll_s < 0:
        raise ValueError("invalid capture parameters")
    limit = int(max_duration_s * sample_rate_hz * 2)
    trailing_limit = int(trailing_silence_s * sample_rate_hz * 2)
    pre_roll_limit = int(pre_roll_s * sample_rate_hz * 2)
    data = bytearray()
    pre_roll = bytearray()
    silence = 0
    started = False
    for chunk in chunks:
        if stop_event is not None and stop_event.is_set():
            break
        if len(chunk) % 2:
            raise ValueError("invalid PCM16 chunk")
        loud = audioop.rms(chunk, 2) / 32768 >= threshold if chunk else False
        if not started:
            if not loud:
                pre_roll.extend(chunk)
                if len(pre_roll) > pre_roll_limit:
                    del pre_roll[:len(pre_roll) - pre_roll_limit]
                continue
            started = True
            data.extend(pre_roll)
        data.extend(chunk)
        silence = 0 if loud else silence + len(chunk)
        if len(data) >= limit or silence >= trailing_limit:
            break
    return AudioClip(bytes(data[:limit]), sample_rate_hz, time.monotonic())


class MicrophoneInput:
    def __init__(self, device=None, sample_rate_hz: int = 16000):
        self.device = device
        self.sample_rate_hz = sample_rate_hz

    def record_once(self, max_duration_s: float, stop_event=None) -> AudioClip:
        try:
            import sounddevice as sd
        except (OSError, ModuleNotFoundError):
            return AlsaMicrophoneInput(self.device, self.sample_rate_hz).record_once(max_duration_s, stop_event)
        if not callable(getattr(sd, "RawInputStream", None)):
            return AlsaMicrophoneInput(self.device, self.sample_rate_hz).record_once(max_duration_s, stop_event)

        block = int(self.sample_rate_hz * 0.02)
        def chunks():
            with sd.RawInputStream(samplerate=self.sample_rate_hz, blocksize=block,
                                   channels=1, dtype="int16", device=self.device) as stream:
                for _ in range(int(max_duration_s / 0.02) + 1):
                    data, overflowed = stream.read(block)
                    if overflowed:
                        raise RuntimeError("microphone overflow")
                    yield bytes(data)
        return capture_until_silence(chunks(), self.sample_rate_hz, max_duration_s,
                                     stop_event=stop_event)


class AlsaMicrophoneInput:
    def __init__(self, device=None, sample_rate_hz: int = 16000, popen=None):
        self.device = device
        self.sample_rate_hz = sample_rate_hz
        self.popen = popen or subprocess.Popen

    def record_once(self, max_duration_s: float, stop_event=None) -> AudioClip:
        command = ["arecord", "-q", "-t", "raw", "-f", "S16_LE",
                   "-r", str(self.sample_rate_hz), "-c", "1"]
        if self.device is not None:
            command.extend(["-D", str(self.device)])
        process = self.popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        block = int(self.sample_rate_hz * 0.02) * 2
        def chunks():
            for _ in range(int(max_duration_s / 0.02) + 1):
                data = process.stdout.read(block)
                if not data:
                    raise RuntimeError("ALSA capture ended")
                yield data
        try:
            return capture_until_silence(chunks(), self.sample_rate_hz, max_duration_s,
                                         stop_event=stop_event)
        finally:
            process.terminate()
            process.wait(timeout=2)
