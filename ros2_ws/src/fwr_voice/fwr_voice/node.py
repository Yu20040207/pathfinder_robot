"""ROS 2 adapter for voice input and high-level tasks."""

from concurrent.futures import ThreadPoolExecutor
import threading
import uuid

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import rclpy
from std_msgs.msg import String

from fwr_interfaces.action import ExecuteRobotTask
from fwr_interfaces.msg import VoiceResponse, VoiceTranscript

from .audio import MicrophoneInput, WavFileInput
from .asr import SherpaSenseVoice
from .gate import GateState
from .intent import parse_local
from .pipeline import VoicePipeline
from .providers import OpenAICompatibleProvider
from .task_slot import TaskSlot
from .tts import DashscopeCosyVoiceTts, FallbackTts, SherpaVitsTts
from .wake import EchoGuard, WakeGate, is_goodbye


def allow_during_playback(text: str, locations) -> bool:
    decision = parse_local(text, locations)
    return decision.request is not None and decision.request.task_type == "STOP"


def log_text(text: str) -> str:
    return text.replace("\n", " ").replace("\r", " ")[:500]


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice_node")
        self.declare_parameter("locations", ["lobby:大厅:一楼大厅", "lab:实验室"])
        self.declare_parameter("asr_model_dir", "")
        self.declare_parameter("tts_model_dir", "")
        self.declare_parameter("tts_speaker_id", 0)
        self.declare_parameter("tts_speed", 1.0)
        self.declare_parameter("tts_streaming", False)
        self.declare_parameter("tts_provider", "local")
        self.declare_parameter("tts_cloud_model", "cosyvoice-v3-flash")
        self.declare_parameter("tts_cloud_voice", "")
        self.declare_parameter("tts_cloud_websocket_url", "")
        self.declare_parameter("cloud_base_url", "")
        self.declare_parameter("cloud_model", "")
        self.declare_parameter("cloud_api_key_env", "FWR_LLM_API_KEY")
        self.declare_parameter("input_wav", "")
        self.declare_parameter("microphone_enabled", False)
        self.declare_parameter("wake_word", "小明同学")
        self.declare_parameter("wake_timeout_s", 30.0)
        self.declare_parameter("simulation", True)
        locations = {}
        for item in self.get_parameter("locations").value:
            location_id, *aliases = item.split(":")
            if not location_id or not aliases:
                raise ValueError("invalid location configuration")
            locations[location_id] = tuple(aliases)
        asr_dir = self.get_parameter("asr_model_dir").value
        tts_dir = self.get_parameter("tts_model_dir").value
        cloud_url = self.get_parameter("cloud_base_url").value
        self.asr = SherpaSenseVoice(asr_dir) if asr_dir else None
        local_tts = (SherpaVitsTts(
            tts_dir,
            speaker_id=int(self.get_parameter("tts_speaker_id").value),
            speed=float(self.get_parameter("tts_speed").value)) if tts_dir else None)
        provider = self.get_parameter("tts_provider").value
        if provider == "aliyun":
            cloud_tts = DashscopeCosyVoiceTts(
                self.get_parameter("tts_cloud_voice").value,
                model=self.get_parameter("tts_cloud_model").value,
                key_env=self.get_parameter("cloud_api_key_env").value,
                websocket_url=self.get_parameter("tts_cloud_websocket_url").value,
                speed=float(self.get_parameter("tts_speed").value))
            self.tts = (FallbackTts(
                cloud_tts, local_tts,
                on_fallback=lambda reason: self.get_logger().warning(
                    f"百炼语音不可用，使用本地音色: {reason}"))
                if local_tts else cloud_tts)
        elif provider == "local":
            self.tts = local_tts
        else:
            raise ValueError("unsupported tts_provider")
        cloud = (OpenAICompatibleProvider(cloud_url, self.get_parameter("cloud_model").value,
                                         self.get_parameter("cloud_api_key_env").value,
                                         timeout_s=8.0)
                 if cloud_url else None)
        self.action = ActionClient(self, ExecuteRobotTask, "/robot/task/execute")
        self.task_slot = TaskSlot()
        self.speaking = threading.Event()
        self.stop_event = threading.Event()
        self.speech_epoch = 0
        self.workers = ThreadPoolExecutor(max_workers=4)
        self.speech_workers = ThreadPoolExecutor(max_workers=1)
        self.speech_generation = 0
        self.tts_streaming = bool(self.get_parameter("tts_streaming").value)
        self.pipeline = VoicePipeline(locations, self._state, cloud, self.asr)
        self.wake_gate = WakeGate(self.get_parameter("wake_word").value,
                                  float(self.get_parameter("wake_timeout_s").value))
        self.echo_guard = EchoGuard()
        self.transcript_pub = self.create_publisher(VoiceTranscript, "/robot/voice/transcript", 10)
        self.response_pub = self.create_publisher(VoiceResponse, "/robot/voice/response", 10)
        self.create_subscription(String, "/robot/voice/text_input", self._on_text, 10)
        self.create_timer(0.5, lambda: self._check_wake_timeout())
        wav = self.get_parameter("input_wav").value
        if wav:
            self.wav_timer = self.create_timer(0.5, lambda: self._consume_wav_once(wav))
        if self.get_parameter("microphone_enabled").value:
            if self.asr is None:
                raise ValueError("microphone requires asr_model_dir")
            self.workers.submit(self._microphone_loop)

    def _state(self):
        return GateState(self.task_slot.is_busy(), self.action.server_is_ready(),
                         bool(self.get_parameter("simulation").value))

    def _publish_response(self, text, status="reply", error_code="", speak=True,
                          log_output=True):
        if self.stop_event.is_set():
            return
        msg = VoiceResponse()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.text, msg.status, msg.error_code = text, status, error_code
        self.response_pub.publish(msg)
        if status != "asleep":
            self.wake_gate.touch()
        if log_output:
            self.get_logger().info(f"输出[{status}]: {log_text(text)}")
        if self.tts and text and speak:
            return self.speech_workers.submit(self._speak, text, self.speech_generation)
        return None

    def _speak(self, text, generation):
        if generation != self.speech_generation or self.stop_event.is_set():
            return
        self.speech_epoch += 1
        self.speaking.set()
        try:
            self.tts.speak(text)
            self.echo_guard.remember(text)
            self.wake_gate.touch()
        except Exception as exc:
            self.get_logger().error(f"TTS failure: {type(exc).__name__}")
        finally:
            self.speaking.clear()
            self.speech_epoch += 1

    def _on_text(self, msg):
        request_id = uuid.uuid4().hex
        local = parse_local(msg.data, self.pipeline.locations)
        if local.request is not None and local.request.task_type == "STOP":
            self._process_text(msg.data, request_id, "text", 0.0)
        else:
            self.workers.submit(self._process_text, msg.data, request_id, "text", 0.0)

    def _process_text(self, text, request_id, provider, latency_ms):
        if provider == "text":
            self.get_logger().info(f"文本输入: {log_text(text)}")
        transcript = VoiceTranscript()
        transcript.header.stamp = self.get_clock().now().to_msg()
        transcript.text, transcript.provider, transcript.latency_ms = text, provider, latency_ms
        self.transcript_pub.publish(transcript)
        partials = []
        def on_partial(sentence):
            if self.tts_streaming:
                partials.append(sentence)
                self._publish_response(sentence, "streaming")
        decision = self.pipeline.handle_text(text, request_id, on_partial=on_partial)
        if decision.kind != "task":
            if decision.kind == "clarify" and decision.reply == "请求已过期。":
                return
            self._publish_response(decision.reply, decision.kind,
                                   speak=not partials or decision.reply == "对话服务暂时不可用，基本指令仍可使用。",
                                   log_output=not partials)
            return
        if decision.request.task_type == "STOP":
            self.speech_generation += 1
            if self.tts:
                self.tts.stop()
            handle = self.task_slot.request_cancel()
            if handle is not None:
                handle.cancel_goal_async()
            self._publish_response(decision.reply, "accepted")
            return
        if decision.request.task_type == "SIM_JOG":
            start_now, handle = self.task_slot.reserve_or_replace(decision.request)
            self._publish_response(decision.reply, "accepted")
            if not start_now:
                if handle is not None:
                    handle.cancel_goal_async()
                return
        else:
            if not self.task_slot.reserve():
                self._publish_response("机器人正忙，请先停止当前任务。", "rejected")
                return
            self._publish_response(decision.reply, "accepted")
        self._send_task(decision.request)

    def _send_task(self, request):
        goal = ExecuteRobotTask.Goal()
        goal.request_id = request.request_id
        goal.task_type = request.task_type
        goal.target_id = request.target_id
        try:
            sent = self.action.send_goal_async(goal)
            sent.add_done_callback(lambda future: self._goal_response(future, goal.task_type))
        except Exception as exc:
            self._finish_task()
            self._publish_response("任务发送失败。", "error", type(exc).__name__)

    def _finish_task(self):
        replacement = self.task_slot.finish()
        if replacement is not None:
            self._send_task(replacement)

    def _goal_response(self, future, task_type=None):
        try:
            handle = future.result()
        except Exception as exc:
            self._finish_task()
            self._publish_response("任务未被接收。", "error", type(exc).__name__)
            return
        if not handle.accepted:
            self._finish_task()
            self._publish_response("任务被拒绝。", "rejected")
            return
        if self.task_slot.accept(handle):
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda future: self._task_result(future, task_type))

    def _task_result(self, future, task_type=None):
        try:
            result = future.result().result
            if not (task_type == "SIM_JOG" and result.status in {"completed", "canceled"}):
                self._publish_response(result.message, result.status)
        except Exception as exc:
            self._publish_response("任务执行结果不可用。", "error", type(exc).__name__)
        finally:
            self._finish_task()

    def _consume_wav_once(self, path):
        self.wav_timer.cancel()
        self.workers.submit(self._process_clip, WavFileInput(path).record_once(15))

    def _check_wake_timeout(self, now=None):
        if self.wake_gate.expire(now):
            self._publish_response("我退下啦，有事再叫我。", "asleep")

    def _process_clip(self, clip, stop_only=False):
        try:
            if self.asr is None:
                raise RuntimeError("ASR model not configured")
            transcript = self.asr.transcribe(clip)
            is_stop = allow_during_playback(transcript.text, self.pipeline.locations)
            if (stop_only or self.speaking.is_set()) and not is_stop:
                return
            if not is_stop and self.echo_guard.is_echo(transcript.text):
                return
            self.get_logger().info(
                f"识别[{transcript.provider}, {transcript.latency_ms:.0f} ms]: {log_text(transcript.text)}")
            command = self.wake_gate.consume(transcript.text)
            if command is None:
                return
            if not command:
                ack = self._publish_response("我在，请说。", "awake")
                if ack is not None:
                    ack.result()
                return
            if is_goodbye(command):
                self.wake_gate.end()
                self._publish_response("好的，我退下啦，有事再叫我。", "asleep")
                return
            self._process_text(command, uuid.uuid4().hex,
                               transcript.provider, transcript.latency_ms)
        except Exception as exc:
            self._publish_response("语音识别失败。", "error", type(exc).__name__)

    def _microphone_loop(self):
        microphone = MicrophoneInput()
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                speech_epoch = self.speech_epoch
                clip = microphone.record_once(15, self.stop_event)
                if clip.pcm16:
                    overlap = self.speaking.is_set() or speech_epoch != self.speech_epoch
                    self._process_clip(clip, stop_only=overlap)
            except Exception as exc:
                self._publish_response("麦克风暂不可用。", "error", type(exc).__name__)
                self.stop_event.wait(1)

    def destroy_node(self):
        self.stop_event.set()
        self.workers.shutdown(wait=False, cancel_futures=True)
        self.speech_workers.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = VoiceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
