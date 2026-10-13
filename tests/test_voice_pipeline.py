from fwr_voice.contracts import TaskRequest
from fwr_voice.dialogue import DialogueResult
from fwr_voice.gate import GateState
from fwr_voice.pipeline import VoicePipeline


class FakeCloud:
    def __init__(self, result=None):
        self.calls = 0
        self.result = result
        self.status_summary = ""

    def respond(self, text, allowed_tasks, status_summary, request_id):
        self.calls += 1
        self.status_summary = status_summary
        return self.result or DialogueResult("你好", None, 1, 1, "fake")


def test_local_command_skips_cloud():
    cloud = FakeCloud()
    pipeline = VoicePipeline({"lobby": ("大厅",)}, lambda: GateState(False, True, True), cloud)
    result = pipeline.handle_text("去大厅", "1")
    assert result.request == TaskRequest("1", "GO_TO_NAMED_LOCATION", "lobby")
    assert cloud.calls == 0
    assert "模拟" in result.reply


def test_chat_never_dispatches_action():
    cloud = FakeCloud()
    pipeline = VoicePipeline({}, lambda: GateState(False, True, True), cloud)
    assert pipeline.handle_text("你好", "1").kind == "reply"
    assert cloud.calls == 1
    assert pipeline.usage_snapshot() == {"calls": 1, "failures": 0, "input_tokens": 1, "output_tokens": 1}


def test_cloud_receives_allowed_location_ids_and_aliases():
    cloud = FakeCloud()
    pipeline = VoicePipeline({"lobby": ("大厅", "一楼大厅")},
                             lambda: GateState(False, True, True), cloud)
    pipeline.handle_text("带我去一楼大厅", "1")
    assert "lobby=大厅/一楼大厅" in cloud.status_summary


def test_stop_invalidates_inflight_cloud_result():
    class Cloud:
        def respond(self, text, allowed_tasks, status_summary, request_id):
            pipeline.handle_text("停止", "stop")
            return DialogueResult("去大厅", TaskRequest(request_id, "GO_TO_NAMED_LOCATION", "lobby"), 1, 1, "fake")

    pipeline = VoicePipeline({"lobby": ("大厅",)}, lambda: GateState(True, True, True), Cloud())
    assert pipeline.handle_text("帮我去大厅", "old").kind == "clarify"


def test_cloud_failure_degrades_to_offline_message():
    class BrokenCloud:
        def respond(self, *args):
            raise TimeoutError()

    pipeline = VoicePipeline({}, lambda: GateState(False, True, True), BrokenCloud())
    assert "暂时" in pipeline.handle_text("你好", "1").reply
    assert pipeline.usage_snapshot()["failures"] == 1
