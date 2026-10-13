from pathlib import Path
import inspect
import time
import threading
from types import SimpleNamespace

from fwr_voice.demo_executor import DemoExecutor
from fwr_voice.node import VoiceNode, allow_during_playback
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from fwr_voice.contracts import TaskDecision
from fwr_voice.wake import WakeGate, EchoGuard


ROOT = Path(__file__).resolve().parents[1]


def test_voice_launch_and_node_never_publish_real_cmd_vel():
    base = ROOT / "ros2_ws/src/fwr_voice/fwr_voice"
    for file in (base / "node.py", base / "demo_executor.py"):
        assert file.exists()
        assert '"/cmd_vel"' not in file.read_text()
    assert '"/demo/cmd_vel"' in (base / "demo_executor.py").read_text()
    launch = ROOT / "ros2_ws/src/fwr_bringup/launch/voice_demo.launch.py"
    assert launch.exists()
    assert "voice_node" in launch.read_text()
    assert '"simulation": True' in launch.read_text()


def test_humble_action_callback_does_not_require_asyncio_loop():
    assert not inspect.iscoroutinefunction(DemoExecutor.execute)


def test_stop_bypasses_cloud_worker_queue():
    events = []
    class Workers:
        def submit(self, *args):
            events.append("queued")
    class Dummy:
        workers = Workers()
        pipeline = type("Pipeline", (), {"locations": {}})()

        def _process_text(self, *args):
            events.append("processed")
    VoiceNode._on_text(Dummy(), String(data="停止"))
    assert events == ["processed"]


def test_playback_overlap_allows_only_stop():
    assert allow_during_playback("停止", {})
    assert not allow_during_playback("你好", {})


def test_streamed_answer_is_spoken_once_as_full_reply():
    published = []
    class Dummy:
        tts_streaming = False
        pipeline = type("Pipeline", (), {
            "handle_text": lambda self, text, request_id, on_partial=None:
                (on_partial("第一句。"), TaskDecision("reply", None, "第一句。第二句。"))[1]
        })()
        transcript_pub = type("Publisher", (), {"publish": lambda self, msg: None})()
        def get_logger(self):
            return SimpleNamespace(info=lambda text: None)
        def get_clock(self):
            return type("Clock", (), {"now": lambda self: type("Now", (), {"to_msg": lambda self: Time()})()})()
        def _publish_response(self, text, status="reply", error_code="", speak=True,
                              log_output=True):
            published.append((text, status, speak, log_output))
    VoiceNode._process_text(Dummy(), "你好", "r", "text", 0.0)
    assert published == [("第一句。第二句。", "reply", True, True)]


def test_microphone_audio_requires_wake_but_stop_bypasses_it():
    events = []
    class Asr:
        def transcribe(self, clip):
            return SimpleNamespace(text=clip, provider="fake", latency_ms=1)
    class Dummy:
        asr = Asr()
        speaking = threading.Event()
        pipeline = SimpleNamespace(locations={})
        wake_gate = WakeGate("小明同学", timeout_s=10)
        echo_guard = EchoGuard()
        def get_logger(self):
            return SimpleNamespace(info=lambda text: None)
        def _process_text(self, text, *args):
            events.append(("command", text))
        def _publish_response(self, text, *args):
            events.append(("response", text))
    dummy = Dummy()
    VoiceNode._process_clip(dummy, "前进一下")
    VoiceNode._process_clip(dummy, "小明同学，后退一下")
    VoiceNode._process_clip(dummy, "停止")
    assert events == [("command", "后退一下"), ("command", "停止")]


def test_wake_ack_finishes_before_next_capture():
    events = []
    class Asr:
        def transcribe(self, clip):
            return SimpleNamespace(text="小明同学", provider="fake", latency_ms=1)
    class Ack:
        def result(self):
            events.append("ack_finished")
    class Dummy:
        asr = Asr()
        speaking = threading.Event()
        pipeline = SimpleNamespace(locations={})
        wake_gate = WakeGate("小明同学", timeout_s=10)
        echo_guard = EchoGuard()
        def get_logger(self):
            return SimpleNamespace(info=lambda text: None)
        def _publish_response(self, text, *args):
            events.append("ack_sent")
            return Ack()
        def _process_text(self, text, *args):
            events.append("command")
    VoiceNode._process_clip(Dummy(), object())
    assert events == ["ack_sent", "ack_finished"]


def test_launch_logs_recognized_speech_and_output():
    logs = []
    class Logger:
        def info(self, text):
            logs.append(text)
    class Asr:
        def transcribe(self, clip):
            return SimpleNamespace(text="小明同学，你好", provider="sensevoice", latency_ms=12)
    class Dummy:
        asr = Asr()
        speaking = threading.Event()
        pipeline = SimpleNamespace(locations={})
        wake_gate = WakeGate("小明同学", timeout_s=10)
        echo_guard = EchoGuard()
        def get_logger(self):
            return Logger()
        def _process_text(self, text, *args):
            pass
        def _publish_response(self, text, *args):
            pass
    VoiceNode._process_clip(Dummy(), object())
    assert any("识别" in line and "小明同学，你好" in line for line in logs)

    class OutputDummy:
        stop_event = threading.Event()
        tts = None
        wake_gate = WakeGate("小明同学", timeout_s=30)
        response_pub = SimpleNamespace(publish=lambda msg: None)
        def get_logger(self):
            return Logger()
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    output = OutputDummy()
    output.wake_gate.consume("小明同学", now=time.monotonic() - 29)
    VoiceNode._publish_response(output, "你好，我在。", "reply")
    assert any("输出" in line and "你好，我在。" in line for line in logs)
    assert output.wake_gate.active_until > time.monotonic() + 25


def test_wake_session_says_goodbye_once_after_30_seconds():
    responses = []
    class Dummy:
        wake_gate = WakeGate("小明同学", timeout_s=30)
        def _publish_response(self, text, status):
            responses.append((text, status))
    dummy = Dummy()
    dummy.wake_gate.consume("小明同学", now=100)
    VoiceNode._check_wake_timeout(dummy, now=129)
    VoiceNode._check_wake_timeout(dummy, now=130)
    VoiceNode._check_wake_timeout(dummy, now=131)
    assert responses == [("我退下啦，有事再叫我。", "asleep")]


def test_recent_tts_echo_does_not_become_new_question():
    commands = []
    class Asr:
        def transcribe(self, clip):
            return SimpleNamespace(text="你好。", provider="sensevoice", latency_ms=10)
    class Dummy:
        asr = Asr()
        speaking = threading.Event()
        pipeline = SimpleNamespace(locations={})
        wake_gate = WakeGate("小明同学", timeout_s=30)
        echo_guard = EchoGuard(hold_s=3)
        def get_logger(self):
            return SimpleNamespace(info=lambda text: None)
        def _process_text(self, text, *args):
            commands.append(text)
        def _publish_response(self, text, *args):
            pass
    dummy = Dummy()
    dummy.wake_gate.consume("小明同学", now=time.monotonic())
    dummy.echo_guard.remember("你好！")
    VoiceNode._process_clip(dummy, object())
    assert commands == []


def test_tts_registers_its_own_audio_as_possible_echo():
    class Dummy:
        speech_generation = 0
        speech_epoch = 0
        stop_event = threading.Event()
        speaking = threading.Event()
        tts = SimpleNamespace(speak=lambda text: None)
        echo_guard = EchoGuard(hold_s=3)
        wake_gate = WakeGate("小明同学", timeout_s=30)
        def get_logger(self):
            return SimpleNamespace(error=lambda text: None)
    dummy = Dummy()
    dummy.wake_gate.consume("小明同学", now=time.monotonic() - 29)
    VoiceNode._speak(dummy, "你好！", 0)
    assert dummy.echo_guard.is_echo("你好。")
    assert dummy.wake_gate.active_until > time.monotonic() + 25


def test_spoken_goodbye_ends_session_without_cloud_reply():
    outputs = []
    commands = []
    class Asr:
        def transcribe(self, clip):
            return SimpleNamespace(text=clip, provider="sensevoice", latency_ms=10)
    class Dummy:
        asr = Asr()
        speaking = threading.Event()
        pipeline = SimpleNamespace(locations={})
        wake_gate = WakeGate("小明同学", timeout_s=30)
        echo_guard = EchoGuard()
        def get_logger(self):
            return SimpleNamespace(info=lambda text: None)
        def _process_text(self, text, *args):
            commands.append(text)
        def _publish_response(self, text, status):
            outputs.append((text, status))
    dummy = Dummy()
    dummy.wake_gate.consume("小明同学", now=time.monotonic())
    VoiceNode._process_clip(dummy, "退一下吧")
    VoiceNode._process_clip(dummy, "今天天气怎么样")
    assert outputs == [("好的，我退下啦，有事再叫我。", "asleep")]
    assert commands == []
