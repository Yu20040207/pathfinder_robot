"""Metrics for a labeled Chinese voice corpus."""

import math


def _distance(a: str, b: str) -> int:
    previous = list(range(len(b) + 1))
    for i, char in enumerate(a, 1):
        current = [i]
        for j, other in enumerate(b, 1):
            current.append(min(current[-1] + 1, previous[j] + 1,
                               previous[j - 1] + (char != other)))
        previous = current
    return previous[-1]


def cer(reference: str, hypothesis: str) -> float:
    if not reference:
        raise ValueError("empty reference text")
    return _distance(reference, hypothesis) / len(reference)


def percentile(values: list[float], p: float) -> float:
    if not values:
        raise ValueError("empty measurements")
    ordered = sorted(values)
    at = (len(ordered) - 1) * p / 100
    low = math.floor(at)
    high = math.ceil(at)
    return ordered[low] + (ordered[high] - ordered[low]) * (at - low)


def summarize(rows: list[dict], input_price_per_million: float = 0,
              output_price_per_million: float = 0) -> dict:
    if not rows:
        raise ValueError("empty corpus")
    references = [row["reference_text"] for row in rows]
    errors = sum(_distance(ref, row["recognized_text"])
                 for ref, row in zip(references, rows))
    chars = sum(map(len, references))
    if not chars:
        raise ValueError("empty references")
    cost = (sum(int(row.get("input_tokens", 0)) for row in rows) * input_price_per_million
            + sum(int(row.get("output_tokens", 0)) for row in rows) * output_price_per_million) / 1e6
    stops = [row for row in rows if row["expected_intent"] == "STOP"]
    terms = [(term, row["recognized_text"]) for row in rows
             for term in str(row.get("terms") or "").split("|") if term]
    return {
        "samples": len(rows), "cer": errors / chars,
        "intent_accuracy": sum(row["expected_intent"] == row["actual_intent"] for row in rows) / len(rows),
        "stop_misses": sum(row["actual_intent"] != "STOP" for row in stops),
        "stop_false_positives": sum(row["actual_intent"] == "STOP" and row["expected_intent"] != "STOP" for row in rows),
        "term_recall": (sum(term in hypothesis for term, hypothesis in terms) / len(terms)
                        if terms else None),
        "latency_p50_ms": percentile([float(row["latency_ms"]) for row in rows], 50),
        "latency_p95_ms": percentile([float(row["latency_ms"]) for row in rows], 95),
        "cloud_cost_per_1000_turns_cny": cost / len(rows) * 1000,
    }
