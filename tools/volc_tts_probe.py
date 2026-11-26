#!/usr/bin/env python3
"""火山引擎（豆包）语音合成接口探测工具。

**用途**：在把云端音色接进 `voice_node` 之前，先用它确认账号、协议、
音色 ID 和返回字段都对得上。它是**独立脚本**，不依赖 ROS，也不改任何状态。

用法：

    export FWR_VOLC_APPID=<控制台 appid>
    export FWR_VOLC_TOKEN=<控制台 access token>

    # 最常用：合成一句话，存成 wav，并打印返回码
    .venv/bin/python tools/volc_tts_probe.py --voice-type BV700_streaming

    # 顺带播放（需要 --device）
    .venv/bin/python tools/volc_tts_probe.py --voice-type <ID> \
        --play --device plughw:2,0

    # 只看原始返回，不解析音频（排查字段用）
    .venv/bin/python tools/volc_tts_probe.py --voice-type <ID> --raw

返回码参考（火山引擎约定）：`3000` 成功；其它值看打印出来的 `message`。
如果这里报 `HTTP 401/403`，通常是 token 不对或服务没开通；
报 `code=3001` 一类，通常是 `voice_type` 和账号开通的服务不匹配。
"""

import argparse
import base64
import json
import os
import sys
import wave
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ros2_ws" / "src" / "fwr_voice"))

from fwr_voice.tts import (  # noqa: E402
    VolcengineTts, _post_json, decode_tts_audio, play_with_alsa)


def main(argv=None):
    parser = argparse.ArgumentParser(description="火山引擎语音合成探测")
    parser.add_argument("--voice-type", default="", help="音色 ID，必填")
    parser.add_argument("--text", default="你好，我是机器人小助手。", help="合成文本")
    parser.add_argument("--endpoint", default=VolcengineTts.DEFAULT_ENDPOINT)
    parser.add_argument("--cluster", default="volcano_tts")
    parser.add_argument("--encoding", default="pcm", choices=["pcm", "wav"])
    parser.add_argument("--sample-rate", type=int, default=24000,
                        help="pcm 模式的采样率（火山常见 24000）")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--appid-env", default="FWR_VOLC_APPID")
    parser.add_argument("--token-env", default="FWR_VOLC_TOKEN")
    parser.add_argument("--save", default="/tmp/volc_tts_probe.wav")
    parser.add_argument("--play", action="store_true", help="合成后播放")
    parser.add_argument("--device", default="", help="ALSA 播放设备，留空走 default")
    parser.add_argument("--raw", action="store_true", help="只打印原始返回，不解析音频")
    args = parser.parse_args(argv)

    if not args.voice_type:
        parser.error("--voice-type 必填（控制台里的音色 ID）")
    appid = os.environ.get(args.appid_env, "").strip()
    token = os.environ.get(args.token_env, "").strip()
    if not appid or not token:
        print(f"缺少凭据：请设置环境变量 {args.appid_env} 和 {args.token_env}")
        return 2

    tts = VolcengineTts(
        args.voice_type, appid_env=args.appid_env, token_env=args.token_env,
        cluster=args.cluster, endpoint=args.endpoint, encoding=args.encoding,
        sample_rate=args.sample_rate, speed=args.speed)
    payload = tts.payload(args.text)
    headers = tts.headers()

    print(f"POST {args.endpoint}")
    print(f"  cluster={args.cluster}  voice_type={args.voice_type}  "
          f"encoding={args.encoding}  speed={args.speed}")
    print(f"  appid={appid[:6]}…  token={token[:4]}…（已脱敏）")
    print(f"  text={args.text!r}")

    status, body = _post_json(args.endpoint, headers, payload)
    print(f"\nHTTP {status}  响应 {len(body)} 字节")
    if args.raw:
        print(body[:2000].decode("utf-8", "replace"))
        return 0 if status == 200 else 1

    try:
        obj = json.loads(body)
    except json.JSONDecodeError:
        print("响应不是 JSON：")
        print(body[:2000].decode("utf-8", "replace"))
        return 1

    print(f"  code={obj.get('code')}  message={obj.get('message')!r}")
    for key in ("reqid", "sequence", "addition"):
        if key in obj:
            print(f"  {key}={obj[key]!r}")
    if status != 200 or obj.get("code") not in (0, 3000):
        print("\n❌ 未成功。上面就是服务端原话，按它调整 appid/token/cluster/voice_type。")
        return 1

    data = obj.get("data") or ""
    if not data:
        print("\n❌ code 是成功但没有 data 字段，实际字段是：", sorted(obj))
        return 1

    raw = base64.b64decode(data)
    print(f"  音频 {len(raw)} 字节（base64 前 {len(data)} 字符）")
    samples, rate = decode_tts_audio(raw, args.encoding, args.sample_rate)
    duration = samples.size / rate if rate else 0
    print(f"  解析后 {samples.size} 采样 / {rate} Hz = {duration:.2f} s")

    out = Path(args.save)
    pcm = (samples * 32767).clip(-32768, 32767).astype("<i2").tobytes()
    with wave.open(str(out), "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(rate)
        handle.writeframes(pcm)
    print(f"  已保存 {out}")

    if args.play:
        import threading
        play_with_alsa(samples, rate, threading.Event(),
                       device=args.device or None)
        print("  已播放")

    print("\n✅ 协议通了。把下面这些填进 launch：")
    print(f"  tts_provider:=volcengine "
          f"tts_volc_voice_type:={args.voice_type} "
          f"tts_volc_cluster:={args.cluster} "
          f"tts_volc_encoding:={args.encoding}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
