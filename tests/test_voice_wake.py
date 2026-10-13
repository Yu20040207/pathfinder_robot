from fwr_voice.wake import WakeGate, EchoGuard, is_goodbye


def test_wake_phrase_and_command_in_same_utterance():
    gate = WakeGate("小明同学", timeout_s=30)
    assert gate.consume("前进一下", now=1) is None
    assert gate.consume("小明同学，前进一下", now=2) == "前进一下"
    assert gate.consume("后退一下", now=3) == "后退一下"
    assert gate.consume("你好", now=32) == "你好"
    assert gate.consume("你好", now=63) is None
    assert gate.expire(now=63)
    assert not gate.expire(now=64)


def test_wake_then_multiple_commands_within_timeout():
    gate = WakeGate("小明同学", timeout_s=30)
    assert gate.consume("小明同学", now=1) == ""
    assert gate.consume("去大厅", now=5) == "去大厅"
    assert gate.consume("你好", now=6) == "你好"
    assert gate.consume("小明同学", now=10) == ""
    assert gate.consume("你好", now=39) == "你好"
    assert gate.consume("你好", now=40) == "你好"
    assert gate.consume("你好", now=70) is None


def test_stop_always_passes_and_keeps_active_session():
    gate = WakeGate("小明同学", timeout_s=10)
    assert gate.consume("停止", now=1) == "停止"
    assert gate.consume("前进", now=1.5) is None
    assert gate.consume("小明同学", now=2) == ""
    assert gate.consume("停止", now=3) == "停止"
    assert gate.consume("前进一下", now=4) == "前进一下"
    assert gate.consume("后退", now=13) == "后退"
    assert gate.consume("右转", now=24) is None


def test_fillers_do_not_trigger_cloud_or_end_session():
    gate = WakeGate("小明同学", timeout_s=30)
    assert gate.consume("小明同学", now=1) == ""
    assert gate.consume("嗯。", now=2) is None
    assert gate.consume("你好", now=3) == "你好"


def test_partial_wake_word_does_not_arm():
    gate = WakeGate("小明同学", timeout_s=10)
    assert gate.consume("小明", now=1) is None
    assert gate.consume("前进一下", now=2) is None


def test_recent_spoken_text_is_recognized_as_echo():
    guard = EchoGuard(hold_s=3)
    guard.remember("你好！", now=1)
    assert guard.is_echo("你好。", now=2)
    assert not guard.is_echo("今天几点？", now=2)
    assert not guard.is_echo("你好。", now=5)


def test_answer_completion_resets_idle_countdown():
    gate = WakeGate("小明同学", timeout_s=30)
    assert gate.consume("小明同学，今天天气如何", now=1) == "今天天气如何"
    assert gate.touch(now=25)
    assert not gate.expire(now=31)
    assert gate.expire(now=55)


def test_goodbye_phrases_include_asr_variant_but_not_motion():
    for phrase in ("退下", "你先退下吧", "退一下吧", "再见", "拜拜啦", "结束对话"):
        assert is_goodbye(phrase)
    for phrase in ("后退一下", "往后退一下", "再见面时再说", "你好"):
        assert not is_goodbye(phrase)


def test_explicit_goodbye_closes_active_session():
    gate = WakeGate("小明同学", timeout_s=30)
    gate.consume("小明同学", now=1)
    assert gate.end()
    assert gate.consume("今天天气怎么样", now=2) is None
    assert not gate.expire(now=31)
