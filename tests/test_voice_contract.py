from pathlib import Path

from fwr_voice.contracts import TaskDecision, TaskRequest, TASK_TYPES


ROOT = Path(__file__).resolve().parents[1]


def test_task_contract_is_limited():
    assert TASK_TYPES == frozenset({"STOP", "QUERY_STATUS", "GO_TO_NAMED_LOCATION", "SIM_JOG"})
    request = TaskRequest("abc", "STOP")
    assert TaskDecision("task", request, "已停止").request == request


def test_ros_interfaces_have_expected_fields():
    base = ROOT / "ros2_ws/src/fwr_interfaces"
    assert "float32 latency_ms" in (base / "msg/VoiceTranscript.msg").read_text()
    assert "string error_code" in (base / "msg/VoiceResponse.msg").read_text()
    action = (base / "action/ExecuteRobotTask.action").read_text()
    assert "string target_id" in action
    assert action.count("---") == 2
