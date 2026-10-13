import pytest

from fwr_voice.benchmark import cer, percentile, summarize


def test_cer_and_empty_reference():
    assert cer("大厅", "大听") == 0.5
    with pytest.raises(ValueError):
        cer("", "你好")


def test_percentile_and_cost_summary():
    assert percentile([100, 200, 300], 50) == 200
    report = summarize([
        {"reference_text": "去大厅", "recognized_text": "去大厅", "expected_intent": "GO_TO_NAMED_LOCATION",
         "actual_intent": "GO_TO_NAMED_LOCATION", "latency_ms": 100, "input_tokens": 10, "output_tokens": 5},
        {"reference_text": "停止", "recognized_text": "停止", "expected_intent": "STOP",
         "actual_intent": "STOP", "latency_ms": 200, "input_tokens": 0, "output_tokens": 0},
    ], input_price_per_million=1, output_price_per_million=2)
    assert report["intent_accuracy"] == 1
    assert report["stop_misses"] == 0
    assert report["cloud_cost_per_1000_turns_cny"] == pytest.approx(0.01)


def test_named_term_recall():
    rows = [{"reference_text": "去一楼大厅", "recognized_text": "去一楼大厅", "expected_intent": "GO_TO_NAMED_LOCATION",
             "actual_intent": "GO_TO_NAMED_LOCATION", "latency_ms": 100, "terms": "一楼大厅|实验室"}]
    assert summarize(rows)["term_recall"] == 0.5
