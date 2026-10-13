"""Replaceable local speech synthesis."""

from pathlib import Path
import os
import subprocess
import threading

import numpy as np


def play_with_alsa(samples, sample_rate: int, interrupt: threading.Event,
                   popen=None) -> None:
    process = (popen or subprocess.Popen)(
        ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", str(sample_rate), "-c", "1"],
        stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
    pcm = (np.clip(samples, -1, 1) * 32767).astype("<i2").tobytes()
    try:
        for index in range(0, len(pcm), 4096):
            if interrupt.is_set():
                process.terminate()
                break
            process.stdin.write(pcm[index:index + 4096])
        try:
            process.stdin.close()
        except BrokenPipeError:
            if not interrupt.is_set():
                raise
        if process.wait(timeout=max(5, len(pcm) / (2 * sample_rate) + 5)) != 0 and not interrupt.is_set():
            raise RuntimeError("ALSA playback failed")
    except (BrokenPipeError, OSError) as exc:
        raise RuntimeError("ALSA playback failed") from exc


class CallbackTts:
    def __init__(self, callback):
        self.callback = callback

    def speak(self, text: str) -> None:
        self.callback(text)


class _AlsaPcmSink:
    def __init__(self):
        self.process = subprocess.Popen(
            ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", "22050", "-c", "1"],
            stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)

    def write(self, data):
        self.process.stdin.write(data)

    def close(self):
        if self.process.poll() is None:
            try:
                self.process.stdin.close()
            except BrokenPipeError:
                pass
            self.process.wait(timeout=10)

    def stop(self):
        if self.process.poll() is None:
            self.process.terminate()


class DashscopeCosyVoiceTts:
    """Stream the selected CosyVoice PCM audio directly to the speaker."""

    def __init__(self, voice: str, *, model: str = "cosyvoice-v3-flash",
                 key_env: str = "FWR_LLM_API_KEY", websocket_url: str = "",
                 speed: float = 1.0, dashscope_module=None, tts_api=None,
                 output_factory=None):
        if not voice or not 0.5 <= speed <= 2.0:
            raise ValueError("invalid cloud voice or speed")
        if websocket_url and not websocket_url.startswith("wss://"):
            raise ValueError("cloud speech endpoint must use WSS")
        self.voice, self.model, self.key_env = voice, model, key_env
        self.websocket_url, self.speed = websocket_url, speed
        self.dashscope_module, self.tts_api = dashscope_module, tts_api
        self.output_factory = output_factory or _AlsaPcmSink
        self.interrupt = threading.Event()
        self.playback_lock = threading.Lock()
        self.active_sink = None
        self.active_synth = None
        self.active_done = None
        self.last_audio_started = False

    def stop(self):
        self.interrupt.set()
        if self.active_done is not None:
            self.active_done.set()
        if self.active_sink is not None and hasattr(self.active_sink, "stop"):
            self.active_sink.stop()
        if self.active_synth is not None and hasattr(self.active_synth, "streaming_cancel"):
            try:
                self.active_synth.streaming_cancel()
            except Exception:
                pass

    def speak(self, text: str):
        key = os.environ.get(self.key_env)
        if not key:
            raise RuntimeError("cloud TTS API key unavailable")
        dashscope = self.dashscope_module
        api = self.tts_api
        if dashscope is None or api is None:
            import dashscope as dashscope
            from dashscope.audio import tts_v2 as api
        with self.playback_lock:
            self.interrupt.clear()
            self.last_audio_started = False
            dashscope.api_key = key
            if self.websocket_url:
                dashscope.base_websocket_api_url = self.websocket_url
            sink = self.output_factory()
            self.active_sink = sink
            done = threading.Event()
            self.active_done = done
            errors = []
            owner = self
            class AudioCallback(api.ResultCallback):
                def on_open(self):
                    pass
                def on_complete(self):
                    done.set()
                def on_error(self, message):
                    errors.append("cloud TTS synthesis failed")
                    done.set()
                def on_close(self):
                    if not done.is_set():
                        errors.append("cloud TTS connection closed")
                    done.set()
                def on_event(self, message):
                    pass
                def on_data(self, data):
                    if data and not owner.interrupt.is_set():
                        owner.last_audio_started = True
                        sink.write(data)
            try:
                synth = api.SpeechSynthesizer(
                    model=self.model, voice=self.voice,
                    format=api.AudioFormat.PCM_22050HZ_MONO_16BIT,
                    speech_rate=self.speed, callback=AudioCallback())
                self.active_synth = synth
                synth.call(text)
                if not done.wait(timeout=30):
                    raise TimeoutError("cloud TTS timed out")
                if errors:
                    raise RuntimeError(errors[0])
                if not self.last_audio_started and not self.interrupt.is_set():
                    raise RuntimeError("cloud TTS returned no audio")
            finally:
                self.active_synth = None
                self.active_sink = None
                self.active_done = None
                sink.close()


class FallbackTts:
    def __init__(self, primary, fallback, on_fallback=None):
        self.primary, self.fallback = primary, fallback
        self.on_fallback = on_fallback

    def stop(self):
        self.primary.stop()
        self.fallback.stop()

    def speak(self, text):
        try:
            self.primary.speak(text)
        except Exception as exc:
            if getattr(self.primary, "last_audio_started", False):
                raise
            if self.on_fallback:
                self.on_fallback(type(exc).__name__)
            self.fallback.speak(text)


class SherpaVitsTts:
    def __init__(self, model_dir: str | Path, device=None, speaker_id: int = 0,
                 speed: float = 1.0):
        import sherpa_onnx

        if not 0.5 <= speed <= 1.5:
            raise ValueError("speed must be between 0.5 and 1.5")
        path = Path(model_dir)
        config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model=str(path / "model.onnx"), tokens=str(path / "tokens.txt"),
                    lexicon=str(path / "lexicon.txt") if (path / "lexicon.txt").exists() else "",
                    data_dir=str(path / "espeak-ng-data") if (path / "espeak-ng-data").exists() else "")))
        if not config.validate():
            raise ValueError("invalid TTS model configuration")
        self.engine = sherpa_onnx.OfflineTts(config)
        if not 0 <= speaker_id < self.engine.num_speakers:
            raise ValueError(f"speaker_id must be between 0 and {self.engine.num_speakers - 1}")
        self.speaker_id = speaker_id
        self.speed = speed
        self.device = device
        self.interrupt = threading.Event()
        self.playback_lock = threading.Lock()

    def stop(self) -> None:
        self.interrupt.set()

    def speak(self, text: str) -> None:
        import sherpa_onnx
        try:
            import sounddevice as sd
        except (OSError, ModuleNotFoundError):
            sd = None

        with self.playback_lock:
            self.interrupt.clear()
            generation = sherpa_onnx.GenerationConfig()
            generation.sid = self.speaker_id
            generation.speed = self.speed
            audio = self.engine.generate(text, generation)
            if len(audio.samples) == 0:
                raise RuntimeError("TTS produced no audio")
            if sd is None or not callable(getattr(sd, "OutputStream", None)):
                play_with_alsa(audio.samples, audio.sample_rate, self.interrupt)
            else:
                with sd.OutputStream(samplerate=audio.sample_rate, channels=1,
                                     dtype="float32", device=self.device) as stream:
                    for index in range(0, len(audio.samples), 2048):
                        if self.interrupt.is_set():
                            break
                        stream.write(audio.samples[index:index + 2048])
