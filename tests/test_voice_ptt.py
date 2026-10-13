from fwr_voice.asr import Transcript
from fwr_voice.ptt import run_once


def test_push_to_talk_waits_then_publishes_transcript():
    order = []
    class Mic:
        def record_once(self, seconds):
            order.append("record")
            return object()
    class Asr:
        def transcribe(self, clip):
            order.append("transcribe")
            return Transcript("去大厅", "fake", 1)
    result = run_once(Mic(), Asr(), lambda text: order.append(text),
                      wait_for_button=lambda: order.append("button"))
    assert result.text == "去大厅"
    assert order == ["button", "record", "transcribe", "去大厅"]
