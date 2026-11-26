"""Replaceable local speech synthesis."""

from pathlib import Path
import base64
import io
import json
import os
import subprocess
import threading
import urllib.error
import urllib.request
import uuid
import wave

import numpy as np


def _alsa_command(rate: int, device=None):
    command = ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", str(rate), "-c", "1"]
    if device:
        command.extend(["-D", str(device)])
    return command


def play_with_alsa(samples, sample_rate: int, interrupt: threading.Event,
                   popen=None, device=None) -> None:
    process = (popen or subprocess.Popen)(
        _alsa_command(sample_rate, device),
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
    def __init__(self, device=None):
        self.process = subprocess.Popen(
            _alsa_command(22050, device),
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
                 output_factory=None, device=None):
        if not voice or not 0.5 <= speed <= 2.0:
            raise ValueError("invalid cloud voice or speed")
        if websocket_url and not websocket_url.startswith("wss://"):
            raise ValueError("cloud speech endpoint must use WSS")
        self.voice, self.model, self.key_env = voice, model, key_env
        self.websocket_url, self.speed = websocket_url, speed
        self.dashscope_module, self.tts_api = dashscope_module, tts_api
        self.output_factory = output_factory or (lambda: _AlsaPcmSink(device))
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


def _post_json(url: str, headers: dict, payload: dict, timeout: float = 20.0):
    """POST JSON 并返回 (HTTP 状态码, 响应体字节)。抽出来是为了单元测试能替换。"""
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(url, data=body, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


def decode_tts_audio(raw: bytes, encoding: str, sample_rate: int):
    """把云端返回的音频解成 (float32 [-1,1] 单声道, 采样率)。

    支持 `pcm`（裸 PCM16LE 单声道）与 `wav`（带头，用头里的采样率）。
    """
    kind = (encoding or "pcm").lower()
    if kind.startswith("wav"):
        with wave.open(io.BytesIO(raw), "rb") as handle:
            rate = handle.getframerate()
            if handle.getsampwidth() != 2 or handle.getnchannels() != 1:
                raise ValueError("only mono 16-bit WAV is supported")
            pcm = handle.readframes(handle.getnframes())
        return np.frombuffer(pcm, dtype="<i2").astype(np.float32) / 32768.0, rate
    if len(raw) % 2:
        raw = raw[:-1]
    return np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0, sample_rate


class VolcengineTts:
    """火山引擎语音合成（豆包同源的云端音色）。

    ⚠️ **协议未经真实账号验证**：按公开的 HTTP JSON 接口形状实现，
    先用 `tools/volc_tts_probe.py` 打通，确认返回码/字段后再用于对话。
    若你的账号走的是「语音合成大模型」的 WebSocket 二进制接口，这里的 HTTP 路径可能不适用。

    需要的环境变量（默认名）：
      `FWR_VOLC_APPID`  —— 火山引擎控制台的 appid
      `FWR_VOLC_TOKEN`  —— access token

    `voice_type` 是音色 ID（如 `BV700_streaming`，大模型是 `zh_female_xxx_bigtts`），
    必须和账号开通的服务对应；`cluster` 通常是 `volcano_tts`。
    """

    DEFAULT_ENDPOINT = "https://openspeech.bytedance.com/api/v1/tts"

    def __init__(self, voice_type: str, *, appid_env: str = "FWR_VOLC_APPID",
                 token_env: str = "FWR_VOLC_TOKEN", cluster: str = "volcano_tts",
                 endpoint: str = DEFAULT_ENDPOINT, encoding: str = "pcm",
                 sample_rate: int = 24000, speed: float = 1.0, uid: str = "fwr-voice",
                 device=None, transport=None):
        if not voice_type:
            raise ValueError("voice_type required")
        if not 0.2 <= speed <= 3.0:
            raise ValueError("speed must be between 0.2 and 3.0")
        if not endpoint.startswith("https://"):
            raise ValueError("volcengine endpoint must use HTTPS")
        if sample_rate <= 0:
            raise ValueError("sample_rate must be positive")
        self.voice_type = voice_type
        self.appid_env, self.token_env = appid_env, token_env
        self.cluster, self.endpoint = cluster, endpoint
        self.encoding, self.sample_rate = encoding, sample_rate
        self.speed, self.uid = speed, uid
        self.device = device
        self.transport = transport or _post_json
        self.interrupt = threading.Event()
        self.playback_lock = threading.Lock()
        self.last_audio_started = False

    def stop(self):
        self.interrupt.set()

    def payload(self, text: str) -> dict:
        # 必须 strip：`$(cat ~/.tok)` 或手抄都可能带空白，带空白会让服务端认不出 grant
        appid = (os.environ.get(self.appid_env) or "").strip()
        token = (os.environ.get(self.token_env) or "").strip()
        if not appid or not token:
            raise RuntimeError("volcengine appid/token unavailable")
        return {
            "app": {"appid": appid, "token": token, "cluster": self.cluster},
            "user": {"uid": self.uid},
            "audio": {"voice_type": self.voice_type, "encoding": self.encoding,
                      "speed_ratio": round(float(self.speed), 2)},
            "request": {"reqid": str(uuid.uuid4()), "text": text,
                        "text_type": "plain", "operation": "query"},
        }

    def headers(self) -> dict:
        token = (os.environ.get(self.token_env) or "").strip()
        # 火山引擎的 Authorization 是 `Bearer;<token>`（分号，不是空格），这是它的历史写法
        return {"Content-Type": "application/json",
                "Authorization": f"Bearer;{token}"}

    def speak(self, text: str):
        if not text.strip():
            raise ValueError("empty text")
        payload = self.payload(text)
        with self.playback_lock:
            self.interrupt.clear()
            self.last_audio_started = False
            status, body = self.transport(self.endpoint, self.headers(), payload)
            if status != 200:
                snippet = body[:200].decode("utf-8", "replace") if body else ""
                raise RuntimeError(f"volcengine TTS HTTP {status} {snippet}")
            try:
                obj = json.loads(body)
            except json.JSONDecodeError as exc:
                raise RuntimeError("volcengine TTS returned non-JSON") from exc
            if not isinstance(obj, dict):
                raise RuntimeError("volcengine TTS returned unexpected payload")
            code = obj.get("code", -1)
            if code not in (0, 3000):
                raise RuntimeError(
                    f"volcengine TTS code={code} {obj.get('message', '')}".strip())
            data = obj.get("data") or ""
            if not data:
                raise RuntimeError("volcengine TTS returned no audio")
            raw = base64.b64decode(data)
            samples, rate = decode_tts_audio(raw, self.encoding, self.sample_rate)
            if samples.size == 0:
                raise RuntimeError("volcengine TTS decoded to empty audio")
            self.last_audio_started = True
            play_with_alsa(samples, rate, self.interrupt, device=self.device)

class SeedTts:
    """豆包语音「语音合成 2.0（Seed-TTS）」HTTP 单向下行接口。

    2026-11-26 用真实账号实测打通，**和经典版接口完全是两套东西**，别混：

    | | 经典版 `VolcengineTts` | 本类 `SeedTts` |
    | --- | --- | --- |
    | 凭据 | appid + access token（body 里 + `Authorization: Bearer;<token>`） | **豆包语音 API Key**，只放 `X-Api-Key` 头 |
    | 资源标识 | body 里 `cluster` | **`X-Api-Resource-Id` 头**（2.0 = `seed-tts-2.0`） |
    | 响应 | 单个 JSON，`data` 是整段 base64 | **NDJSON 流**：多块 `{"code":0,"data":...}`，`code=20000000` 收尾 |
    | 音色名 | `BV700_streaming` 等 | `saturn_*_tob` 这一族 |

    ⚠️ 三个实测踩过的坑：
    1. 把 API Key 当 access token 塞进 `Authorization` → `invalid auth token`；
       塞进 body 的 `app.token` → `load grant: requested grant not found`。
    2. 不带 `X-Api-Resource-Id` → `get resource id empty`；
       用了没开通的资源（如 `seed-tts-3.0`）→ `requested resource not granted`。
    3. 音色和资源不匹配（拿 `*_bigtts` 喂 2.0）→
       `resource ID is mismatched with speaker related resource`（HTTP 200 但 code 非 0）。

    凭据环境变量默认 `FWR_VOLC_APIKEY`（就是控制台「API Key 管理」里那串 UUID）。
    """

    DEFAULT_ENDPOINT = "https://openspeech.bytedance.com/api/v3/tts/unidirectional"
    DEFAULT_RESOURCE_ID = "seed-tts-2.0"
    # 流结束标记：不是错误，最后一块 code 是这个值、通常不带 data
    END_CODE = 20000000

    def __init__(self, voice: str, *, apikey_env: str = "FWR_VOLC_APIKEY",
                 resource_id: str = DEFAULT_RESOURCE_ID,
                 endpoint: str = DEFAULT_ENDPOINT, encoding: str = "pcm",
                 sample_rate: int = 24000, uid: str = "fwr-voice",
                 device=None, transport=None):
        if not voice:
            raise ValueError("voice required")
        if not endpoint.startswith("https://"):
            raise ValueError("seed tts endpoint must use HTTPS")
        if sample_rate <= 0:
            raise ValueError("sample_rate must be positive")
        self.voice, self.apikey_env = voice, apikey_env
        self.resource_id, self.endpoint = resource_id, endpoint
        self.encoding, self.sample_rate = encoding, sample_rate
        self.uid = uid
        self.device = device
        self.transport = transport or _post_json
        self.interrupt = threading.Event()
        self.playback_lock = threading.Lock()
        self.last_audio_started = False

    def stop(self):
        self.interrupt.set()

    def api_key(self) -> str:
        key = (os.environ.get(self.apikey_env) or "").strip()
        if not key:
            raise RuntimeError(f"doubao TTS api key unavailable ({self.apikey_env})")
        return key

    def payload(self, text: str) -> dict:
        return {
            "user": {"uid": self.uid},
            "req_params": {"text": text, "speaker": self.voice,
                           "audio_params": {"format": self.encoding,
                                            "sample_rate": self.sample_rate}},
        }

    def headers(self) -> dict:
        return {"Content-Type": "application/json",
                "X-Api-Key": self.api_key(),
                "X-Api-Resource-Id": self.resource_id}

    @classmethod
    def parse_stream(cls, body):
        """把 NDJSON 响应拆成 (PCM 字节, 错误列表)。

        一块一行；`code=0` 带 data 的是音频；`code=20000000` 是正常结束。
        其余 code 收进错误列表（HTTP 200 也可能是业务错误，必须看 code）。
        """
        audio, errors = [], []
        for line in body.decode("utf-8", "replace").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(obj, dict):
                continue
            code = obj.get("code", -1)
            if code in (0, cls.END_CODE):
                data = obj.get("data")
                if data:
                    audio.append(base64.b64decode(data))
                continue
            errors.append(f"code={code} {obj.get('message', '')}".strip())
        return b"".join(audio), errors

    def speak(self, text: str):
        if not text.strip():
            raise ValueError("empty text")
        with self.playback_lock:
            self.interrupt.clear()
            self.last_audio_started = False
            status, body = self.transport(self.endpoint, self.headers(),
                                          self.payload(text))
            raw, errors = self.parse_stream(body) if status == 200 else (b"", [])
            if status != 200:
                snippet = body[:200].decode("utf-8", "replace") if body else ""
                raise RuntimeError(f"doubao TTS HTTP {status} {snippet}")
            if errors:
                raise RuntimeError("doubao TTS " + "; ".join(errors))
            if not raw:
                raise RuntimeError("doubao TTS returned no audio")
            samples, rate = decode_tts_audio(raw, self.encoding, self.sample_rate)
            if samples.size == 0:
                raise RuntimeError("doubao TTS decoded to empty audio")
            self.last_audio_started = True
            play_with_alsa(samples, rate, self.interrupt, device=self.device)


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
                play_with_alsa(audio.samples, audio.sample_rate, self.interrupt,
                               device=self.device)
            else:
                with sd.OutputStream(samplerate=audio.sample_rate, channels=1,
                                     dtype="float32", device=self.device) as stream:
                    for index in range(0, len(audio.samples), 2048):
                        if self.interrupt.is_set():
                            break
                        stream.write(audio.samples[index:index + 2048])
