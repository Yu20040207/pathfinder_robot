#!/usr/bin/env python3
"""火山方舟（Ark）语音合成 —— 多端点探测工具。

**为什么需要它**：豆包音色是云端服务，而火山有两套完全不同的接口
（方舟 Ark 的 OpenAI 兼容 HTTP、语音技术的私有大模型 WebSocket）。
没有官方文档在同一台机器上可查时，**让它自己去问服务端**最快：
每个候选端点都打一次，把真实状态码和服务端原话打出来。

用法：

    export ARK_API_KEY=<控制台「API Key 管理」里创建的 key>

    .venv/bin/python tools/ark_tts_probe.py \
        --voice multi_female_vv_xi_v3_uranus_bigtts \
        --model doubao-audio-1-0

    # 只试某一个候选
    .venv/bin/python tools/ark_tts_probe.py --only ark-speech ...

    # 用控制台示例里给的完整 JSON（最准，直接把示例抄进来）
    .venv/bin/python tools/ark_tts_probe.py --url <url> --json '{"model":"...",...}'

成功的判据：**返回体里出现音频**（`Content-Type: audio/*`，或 JSON 里 `data` 是 base64）。
脚本会把音频存成 WAV，并打印能直接抄进 launch 的参数。
"""

import argparse
import base64
import json
import os
import sys
import wave
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ros2_ws" / "src" / "fwr_voice"))

from fwr_voice.tts import _post_json, decode_tts_audio  # noqa: E402

ARK_BASE = "https://ark.cn-beijing.volces.com/api/v3"
DEFAULT_TEXT = "你好，我是机器人小助手。"


def candidates(voice, model, text, appid, token):
    """按"最可能 → 最不可能"排的候选请求。"""
    items = [
        # ① 方舟 OpenAI 兼容的 /audio/speech（最可能：你在 Ark 控制台选的音色）
        ("ark-speech", f"{ARK_BASE}/audio/speech",
         {"Authorization": f"Bearer {os.environ.get('ARK_API_KEY', '')}",
          "Content-Type": "application/json"},
         {"model": model, "input": text, "voice": voice,
          "response_format": "pcm", "speed": 1.0}),
        # ② 同一个端点，字段名换成 text / audio（不同版本命名不一致）
        ("ark-speech-text", f"{ARK_BASE}/audio/speech",
         {"Authorization": f"Bearer {os.environ.get('ARK_API_KEY', '')}",
          "Content-Type": "application/json"},
         {"model": model, "text": text, "voice": voice,
          "response_format": "pcm"}),
        # ③ 语音技术「大模型」HTTP 版（需要 appid + token，不是 Ark key）
        ("openspeech-v3", "https://openspeech.bytedance.com/api/v3/tts/unidirectional",
         {"X-Api-App-Id": appid or "", "X-Api-Access-Key": token or "",
          "X-Api-Resource-Id": "seed-tts-1.0", "Content-Type": "application/json"},
         {"user": {"uid": "fwr-voice"},
          "req_params": {"text": text, "speaker": voice,
                         "audio_params": {"format": "pcm", "sample_rate": 24000}}}),
        # ④ 老的标准版 HTTP（cluster volcano_tts）
        ("openspeech-v1", "https://openspeech.bytedance.com/api/v1/tts",
         {"Authorization": f"Bearer;{token or ''}", "Content-Type": "application/json"},
         {"app": {"appid": appid or "", "token": token or "", "cluster": "volcano_tts"},
          "user": {"uid": "fwr-voice"},
          "audio": {"voice_type": voice, "encoding": "pcm"},
          "request": {"reqid": "probe", "text": text,
                      "text_type": "plain", "operation": "query"}}),
    ]
    return items


def save_audio(raw, out_path, sample_rate, encoding="pcm"):
    samples, rate = decode_tts_audio(raw, encoding, sample_rate)
    if samples.size == 0:
        return None
    pcm = (samples * 32767).clip(-32768, 32767).astype("<i2").tobytes()
    with wave.open(str(out_path), "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(rate)
        handle.writeframes(pcm)
    return samples.size / rate


def looks_like_audio(status, headers, body, content_type):
    if body[:1] in (b"R", b"W") or body[:4] == b"\x00\x00\x00\x00":
        return "raw"
    if content_type and content_type.split(";")[0].strip().startswith("audio/"):
        return "raw"
    if body[:2] in (b"ID", b"RI") and body[8:12] == b"WAVE":
        return "raw"
    if status == 200:
        try:
            obj = json.loads(body)
        except (json.JSONDecodeError, ValueError):
            return None
        if isinstance(obj, dict):
            if isinstance(obj.get("data"), str) and len(obj["data"]) > 200:
                return "b64"
            for key in ("audio", "audio_data"):
                if isinstance(obj.get(key), str) and len(obj[key]) > 200:
                    return "b64"
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description="火山方舟语音合成多端点探测")
    parser.add_argument("--voice", default="multi_female_vv_xi_v3_uranus_bigtts")
    parser.add_argument("--model", default="doubao-audio-1-0",
                        help="方舟模型 ID 或推理接入点 ep-xxxx")
    parser.add_argument("--text", default=DEFAULT_TEXT)
    parser.add_argument("--only", default="", help="只试指定候选名")
    parser.add_argument("--url", default="", help="自定义 URL（配合 --json）")
    parser.add_argument("--json", default="", help="自定义请求体 JSON")
    parser.add_argument("--auth", default="", help="自定义 Authorization 头")
    parser.add_argument("--sample-rate", type=int, default=24000)
    parser.add_argument("--save", default="/tmp/ark_tts_probe.wav")
    args = parser.parse_args(argv)

    appid = os.environ.get("FWR_VOLC_APPID", "").strip()
    token = os.environ.get("FWR_VOLC_TOKEN", "").strip()
    ark_key = os.environ.get("ARK_API_KEY", "").strip()

    print("凭据检查：")
    print(f"  ARK_API_KEY      {'已设置 ' + ark_key[:6] + '…' if ark_key else '未设置'}")
    print(f"  FWR_VOLC_APPID   {'已设置' if appid else '未设置'}")
    print(f"  FWR_VOLC_TOKEN   {'已设置' if token else '未设置'}")
    print()

    if args.url:
        if not args.json:
            parser.error("--url 必须配合 --json")
        headers = {"Content-Type": "application/json"}
        if args.auth:
            headers["Authorization"] = args.auth
        attempts = [("custom", args.url, headers, json.loads(args.json))]
    else:
        attempts = candidates(args.voice, args.model, args.text, appid, token)
        if args.only:
            attempts = [a for a in attempts if a[0] == args.only]
            if not attempts:
                parser.error(f"没有名为 {args.only} 的候选")

    success = None
    for name, url, headers, payload in attempts:
        if not any(v for k, v in headers.items() if k.lower() in
                   ("authorization", "x-api-app-id")):
            print(f"— 跳过 {name}：没有可用凭据")
            continue
        print(f"=== {name} ===")
        print(f"POST {url}")
        try:
            status, body = _post_json(url, headers, payload, timeout=25)
        except Exception as exc:  # 网络层失败也要打印，别静默
            print(f"  请求失败：{type(exc).__name__}: {exc}\n")
            continue
        content_type = ""
        print(f"  HTTP {status}  {len(body)} 字节")
        snippet = body[:400].decode("utf-8", "replace")
        print(f"  返回：{snippet}{'…' if len(body) > 400 else ''}")
        kind = looks_like_audio(status, {}, body, content_type)
        if kind:
            raw = body if kind == "raw" else base64.b64decode(
                json.loads(body).get("data") or json.loads(body).get("audio"))
            duration = save_audio(raw, args.save, args.sample_rate)
            if duration:
                print(f"  ✅ 拿到音频 {duration:.2f} s → {args.save}")
                success = (name, url, payload)
                break
        print()

    if not success:
        print("❌ 没有一个候选成功。把上面每一段的 HTTP 状态和返回原话发回来，")
        print("   尤其是 404（路径不对）/ 401（凭据不对）/ 400（字段名不对）的区别——")
        print("   它们分别指向完全不同的修法。")
        return 1

    name, url, payload = success
    print(f"\n✅ 走通的是 {name}：{url}")
    print("把这几行的字段记下来，我据此写 ArkTts：")
    for key in ("model", "voice", "input", "response_format", "text"):
        if key in payload:
            print(f"  {key} = {payload[key]!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
