#!/usr/bin/env python3
"""豆包语音（火山引擎语音合成）音色批量试听。

**用途**：控制台「音色库」里有几十上百个音色，网页上一个一个点很慢，而且没法对比。
这个脚本把一批 `voice_type` 用同一句话合成出来，生成一个本地 HTML 试听页
（和本地 187 音色那套一样的用法），挑好了再把编号写进 launch。

用法：

    export FWR_VOLC_APIKEY=$(cat ~/.volc_token)   # 豆包语音控制台「API Key」里那串 UUID

    # 直接列几个
    .venv/bin/python tools/volc_voice_audition.py \
        --voices zh_female_xxx_bigtts,zh_male_yyy_bigtts

    # 或者从文件读（每行一个，# 后面是注释）
    .venv/bin/python tools/volc_voice_audition.py --voices-file ~/volc_voices.txt

    # 换试听文本 / 换目录
    .venv/bin/python tools/volc_voice_audition.py --voices-file ~/v.txt \
        --text "我已经转向你了，请说指令。" --out models/voice/audition_cloud

⚠️ 会消耗你的字数额度（每个音色 = 一句）。脚本结束会打印总字数。
"""

import argparse
import base64
import html
import os
import sys
import wave
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ros2_ws" / "src" / "fwr_voice"))

from fwr_voice import tts as tts_module  # noqa: E402
from fwr_voice.tts import SeedTts  # noqa: E402


def read_voices(args):
    items = []
    if args.voices:
        items.extend(v.strip() for v in args.voices.split(",") if v.strip())
    if args.voices_file:
        path = Path(args.voices_file).expanduser()
        if not path.is_file():
            raise SystemExit(f"找不到音色列表文件：{path}")
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.split("#", 1)[0].strip()
            if line:
                items.extend(v.strip() for v in line.split(",") if v.strip())
    seen, unique = set(), []
    for voice in items:
        if voice not in seen:
            seen.add(voice)
            unique.append(voice)
    return unique


def safe_name(voice):
    return "".join(c if c.isalnum() or c in "-_." else "_" for c in voice)


def decode_wav_file(path):
    """把刚写出的 WAV 读回成 (float32 单声道, 采样率)，供 --play 用。"""
    import numpy as np
    with wave.open(str(path), "rb") as handle:
        rate = handle.getframerate()
        frames = handle.readframes(handle.getnframes())
    samples = np.frombuffer(frames, dtype="<i2").astype(np.float32) / 32768.0
    return samples, rate


def main(argv=None):
    parser = argparse.ArgumentParser(description="豆包语音音色批量试听")
    parser.add_argument("--voices", default="", help="逗号分隔的 voice_type")
    parser.add_argument("--voices-file", default="", help="每行一个 voice_type 的文件")
    parser.add_argument("--text", default="你好，我是机器人小助手。")
    parser.add_argument("--out", default="models/voice/audition_cloud")
    parser.add_argument("--resource-id", default=SeedTts.DEFAULT_RESOURCE_ID,
                        help="豆包语音资源 ID（语音合成 2.0 = seed-tts-2.0）")
    parser.add_argument("--encoding", default="pcm")
    parser.add_argument("--sample-rate", type=int, default=24000)
    parser.add_argument("--endpoint", default=SeedTts.DEFAULT_ENDPOINT)
    parser.add_argument("--play", action="store_true",
                        help="合成完直接按顺序播出来（不用开浏览器）")
    parser.add_argument("--device", default="",
                        help="播放设备，如 plughw:2,0；空 = ALSA default")
    args = parser.parse_args(argv)

    voices = read_voices(args)
    if not voices:
        parser.error("没有音色：用 --voices 或 --voices-file 指定")

    if not (os.environ.get("FWR_VOLC_APIKEY") or "").strip():
        raise SystemExit("缺少凭据：请设置 FWR_VOLC_APIKEY（豆包语音 API Key）")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"共 {len(voices)} 个音色，试听文本 {len(args.text)} 字")
    print(f"预计消耗 {len(voices) * len(args.text)} 字额度\n")

    rows, failures = [], []
    for index, voice in enumerate(voices, 1):
        # 用 SeedTts 自身走真实协议，顺便验证这个音色到底属于哪个资源：
        # 属于别的资源会返回 code=55000000 "resource ID is mismatched…"
        tts = SeedTts(voice, resource_id=args.resource_id, endpoint=args.endpoint,
                      encoding=args.encoding, sample_rate=args.sample_rate)
        captured = []
        original_play = tts_module.play_with_alsa
        tts_module.play_with_alsa = lambda samples, rate, interrupt, **kw: captured.append(
            (samples, rate))
        try:
            tts.speak(args.text)
        except Exception as exc:
            failures.append((voice, str(exc)[:110]))
            print(f"[{index}/{len(voices)}] ✗ {voice}  {str(exc)[:80]}")
            continue
        finally:
            tts_module.play_with_alsa = original_play
        if not captured:
            failures.append((voice, "未产生音频"))
            print(f"[{index}/{len(voices)}] ✗ {voice}  未产生音频")
            continue
        samples, rate = captured[0]
        name = f"{index:02d}_{safe_name(voice)}.wav"
        pcm = (samples * 32767).clip(-32768, 32767).astype("<i2").tobytes()
        with wave.open(str(out_dir / name), "wb") as handle:
            handle.setnchannels(1)
            handle.setsampwidth(2)
            handle.setframerate(rate)
            handle.writeframes(pcm)
        duration = samples.size / rate
        rows.append((index, voice, name, duration))
        print(f"[{index}/{len(voices)}] ✓ {voice}  {duration:.2f}s")

    if rows:
        parts = ["<meta charset='utf-8'><title>豆包音色试听</title>",
                 "<style>body{font-family:sans-serif;margin:20px;background:#111;color:#eee}",
                 "div.r{display:flex;align-items:center;gap:12px;margin:3px 0}",
                 "b{width:40px;color:#7cf}code{color:#fc7;font-size:12px}",
                 "audio{height:28px}span{color:#888;font-size:12px}</style>",
                 f"<h2>豆包语音音色试听（{len(rows)} 个）</h2>",
                 f"<p>{html.escape(args.text)}　选定后传 "
                 f"<code>tts_provider:=doubao tts_seed_voice:=&lt;voice_type&gt;</code></p>"]
        for index, voice, name, duration in rows:
            # 内嵌 base64：整页自包含，复制到哪都能放（相对路径在文件卡片里会失效）
            payload = base64.b64encode((out_dir / name).read_bytes()).decode()
            parts.append(
                f"<div class='r'><b>#{index}</b><audio controls preload='metadata' "
                f"src='data:audio/wav;base64,{payload}'></audio>"
                f"<code>{html.escape(voice)}</code><span>{duration:.2f}s</span></div>")
        (out_dir / "index.html").write_text("\n".join(parts), encoding="utf-8")
        print(f"\n试听页：{out_dir / 'index.html'}")

    if failures:
        print(f"\n失败 {len(failures)} 个（不是所有音色都属于已开通的服务）：")
        for voice, reason in failures:
            print(f"  {voice}: {reason}")
        print("\n`resource ID is mismatched` = 这个音色不属于当前资源。"
              "\n语音合成 2.0 的音色是 `saturn_*_tob` 这一族；"
              "`*_bigtts` / `*_v3_*` 属于别的资源（3.0 未开通就用不了）。")

    if args.play and rows:
        print("\n按顺序播放（Ctrl-C 跳过剩余）：")
        from fwr_voice.tts import play_with_alsa
        import threading as _th
        for index, voice, name, _duration in rows:
            print(f"  #{index} {voice}")
            _s, _r = decode_wav_file(out_dir / name)
            try:
                play_with_alsa(_s, _r, _th.Event(),
                               device=args.device or None)
            except Exception as exc:
                print(f"    播放失败：{type(exc).__name__}: {exc}")
                break

    ok = len(rows)
    print(f"\n完成：成功 {ok} / {len(voices)}，实耗约 {ok * len(args.text)} 字额度")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
