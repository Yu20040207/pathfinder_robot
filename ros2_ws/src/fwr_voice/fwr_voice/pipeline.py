"""Voice decision pipeline independent of ROS 2."""

from dataclasses import replace
import threading

from .contracts import CLOUD_TASK_TYPES, TaskDecision
from .dialogue import route_cloud
from .intent import parse_local
from .gate import authorize


class VoicePipeline:
    def __init__(self, locations, get_state, dialogue=None, asr=None):
        self.locations = locations
        self.get_state = get_state
        self.dialogue = dialogue
        self.asr = asr
        self._lock = threading.Lock()
        self._generation = 0
        self._usage = {"calls": 0, "failures": 0, "input_tokens": 0, "output_tokens": 0}

    def usage_snapshot(self) -> dict[str, int]:
        with self._lock:
            return dict(self._usage)

    def handle_text(self, text: str, request_id: str, on_partial=None) -> TaskDecision:
        local = parse_local(text, self.locations)
        with self._lock:
            self._generation += 1
            generation = self._generation
        if local.request is not None:
            request = replace(local.request, request_id=request_id)
            return authorize(request, self.get_state(), set(self.locations))
        if local.kind == "clarify":
            return local
        if self.dialogue is None:
            return TaskDecision("reply", None, "当前离线，暂时无法进行开放对话。")
        with self._lock:
            self._usage["calls"] += 1
        try:
            if hasattr(self.dialogue, "answer_stream"):
                def cancelled():
                    with self._lock:
                        return generation != self._generation

                def emit(sentence):
                    if on_partial and not cancelled():
                        on_partial(sentence)

                result = self.dialogue.answer_stream(text, on_sentence=emit,
                                                     cancelled=cancelled)
                with self._lock:
                    self._usage["input_tokens"] += result.input_tokens
                    self._usage["output_tokens"] += result.output_tokens
                    if generation != self._generation:
                        return TaskDecision("clarify", None, "请求已过期。")
                return TaskDecision("reply", None, result.text)
            locations = ",".join(f"{location_id}={'/'.join(aliases)}"
                                 for location_id, aliases in sorted(self.locations.items()))
            status = "忙碌" if self.get_state().busy else "空闲"
            summary = f"{status};可用地点:{locations or '无'}"
            result = self.dialogue.respond(text, tuple(sorted(CLOUD_TASK_TYPES)),
                                           summary, request_id)
        except Exception:
            with self._lock:
                self._usage["failures"] += 1
            return TaskDecision("reply", None, "对话服务暂时不可用，基本指令仍可使用。")
        with self._lock:
            self._usage["input_tokens"] += result.input_tokens
            self._usage["output_tokens"] += result.output_tokens
            if generation != self._generation:
                return TaskDecision("clarify", None, "请求已过期。")
        return route_cloud(result, request_id, self.get_state(), set(self.locations))

    def handle_clip(self, clip, request_id: str) -> TaskDecision:
        if self.asr is None:
            raise RuntimeError("ASR not configured")
        transcript = self.asr.transcribe(clip)
        return self.handle_text(transcript.text, request_id)
