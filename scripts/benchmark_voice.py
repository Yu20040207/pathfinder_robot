"""Run the same labeled WAV corpus against local ASR and intent routing."""

import argparse
import csv
import json
from pathlib import Path

from fwr_voice.asr import SherpaSenseVoice
from fwr_voice.audio import WavFileInput
from fwr_voice.benchmark import summarize
from fwr_voice.intent import parse_local


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--locations-json", type=Path)
    parser.add_argument("--output", type=Path, default=Path("voice-benchmark.json"))
    parser.add_argument("--input-price-per-million", type=float, default=0)
    parser.add_argument("--output-price-per-million", type=float, default=0)
    args = parser.parse_args()
    locations = json.loads(args.locations_json.read_text()) if args.locations_json else {}
    asr = SherpaSenseVoice(args.model_dir)
    rows = []
    with args.manifest.open(newline="", encoding="utf-8") as stream:
        for item in csv.DictReader(stream):
            path = Path(item["audio_path"])
            if not path.is_absolute():
                path = args.manifest.parent / path
            transcript = asr.transcribe(WavFileInput(path).record_once(15))
            decision = parse_local(transcript.text, locations)
            rows.append({**item, "recognized_text": transcript.text,
                         "actual_intent": decision.request.task_type if decision.request else "NONE",
                         "latency_ms": transcript.latency_ms})
    summary = summarize(rows, args.input_price_per_million, args.output_price_per_million)
    args.output.write_text(json.dumps({"summary": summary, "rows": rows}, ensure_ascii=False, indent=2))
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
