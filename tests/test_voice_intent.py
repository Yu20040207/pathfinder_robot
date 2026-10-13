from fwr_voice.intent import parse_local


LOCATIONS = {"lobby": ("大厅", "一楼大厅"), "lab": ("实验室",)}


def test_stop_preempts_other_words():
    assert parse_local("停止，别去大厅", LOCATIONS).request.task_type == "STOP"
    assert parse_local("机器人正忙，请先停止当前任务。", LOCATIONS).request is None


def test_status_and_named_location():
    assert parse_local("机器人状态", LOCATIONS).request.task_type == "QUERY_STATUS"
    decision = parse_local("去一楼大厅", LOCATIONS)
    assert decision.request.target_id == "lobby"


def test_unknown_or_ambiguous_location_is_not_motion():
    assert parse_local("去仓库", LOCATIONS).kind == "clarify"
    assert parse_local("去大厅", {"a": ("大厅",), "b": ("大厅",)}).kind == "clarify"
    assert parse_local("", LOCATIONS).kind == "clarify"
