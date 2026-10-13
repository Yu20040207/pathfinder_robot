"""OpenAI-compatible text-only chat adapter."""

import os
import json
import re
import time
from dataclasses import dataclass

import requests

from .dialogue import DialogueResult, parse_dialogue


@dataclass(frozen=True)
class AnswerResult:
    text: str
    input_tokens: int = 0
    output_tokens: int = 0
    first_token_latency_ms: float = 0.0
    total_latency_ms: float = 0.0


class OpenAICompatibleProvider:
    def __init__(self, base_url: str, model: str, api_key_env: str,
                 timeout_s: float = 3.0, max_output_tokens: int = 256,
                 post=None):
        if not base_url.startswith("https://"):
            raise ValueError("cloud endpoint must use HTTPS")
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.api_key_env = api_key_env
        self.timeout_s = timeout_s
        self.max_output_tokens = max_output_tokens
        self.post = post or requests.post

    def answer_stream(self, text: str, on_sentence=None, cancelled=None) -> AnswerResult:
        key = os.environ.get(self.api_key_env)
        if not key:
            raise RuntimeError(f"missing API key in {self.api_key_env}")
        started = time.monotonic()
        response = self.post(
            self.base_url + "/chat/completions",
            headers={"Authorization": f"Bearer {key}", "Content-Type": "application/json"},
            json={"model": self.model, "messages": [
                {"role": "system", "content": "你是机器人的中文语音助手。简短、准确地回答问题。不要输出 JSON、控制指令或动作承诺。"},
                {"role": "user", "content": text}],
                "temperature": 0.2, "max_tokens": self.max_output_tokens,
                "stream": True, "stream_options": {"include_usage": True},
                "enable_thinking": False},
            stream=True, timeout=(2, self.timeout_s))
        try:
            response.raise_for_status()
            content = ""
            pending = ""
            first_ms = 0.0
            input_tokens = output_tokens = 0
            for line in response.iter_lines(decode_unicode=True):
                if cancelled and cancelled():
                    break
                if isinstance(line, bytes):
                    line = line.decode("utf-8")
                if not line.startswith("data: "):
                    continue
                payload = line[6:].strip()
                if payload == "[DONE]":
                    break
                chunk = json.loads(payload)
                usage = chunk.get("usage") or {}
                input_tokens = int(usage.get("prompt_tokens") or input_tokens)
                output_tokens = int(usage.get("completion_tokens") or output_tokens)
                choices = chunk.get("choices") or []
                piece = (choices[0].get("delta") or {}).get("content") if choices else None
                if not isinstance(piece, str) or not piece:
                    continue
                if not first_ms:
                    first_ms = (time.monotonic() - started) * 1000
                content += piece
                pending += piece
                if len(content) > 1200:
                    break
                while True:
                    boundary = re.search(r"[。！？!?；;\n]", pending)
                    if boundary is None:
                        break
                    sentence = pending[:boundary.end()].strip()
                    pending = pending[boundary.end():]
                    if sentence and on_sentence:
                        on_sentence(sentence)
            if pending.strip() and on_sentence and not (cancelled and cancelled()):
                on_sentence(pending.strip())
            if not content.strip() and not (cancelled and cancelled()):
                raise ValueError("empty streamed answer")
            return AnswerResult(content.strip(), input_tokens, output_tokens,
                                first_ms, (time.monotonic() - started) * 1000)
        finally:
            response.close()

    def respond(self, text: str, allowed_tasks: tuple[str, ...],
                status_summary: str, request_id: str) -> DialogueResult:
        key = os.environ.get(self.api_key_env)
        if not key:
            raise RuntimeError(f"missing API key in {self.api_key_env}")
        instruction = ("你是机器人的中文语音助手。只输出 JSON 对象，恰好包含 reply 和 task。"
                       "task 为 null 或恰好包含 task_type、target_id 两个字符串字段。"
                       f"允许任务：{','.join(allowed_tasks)}。禁止坐标、速度、ROS 话题和其他字段。"
                       f"机器人状态：{status_summary}。普通对话用 task:null。")
        response = self.post(
            self.base_url + "/chat/completions",
            headers={"Authorization": f"Bearer {key}", "Content-Type": "application/json"},
            json={"model": self.model, "messages": [
                {"role": "system", "content": instruction},
                {"role": "user", "content": text}],
                "temperature": 0, "max_tokens": self.max_output_tokens,
                "response_format": {"type": "json_object"}},
            timeout=self.timeout_s)
        response.raise_for_status()
        body = response.json()
        usage = body.get("usage") or {}
        return parse_dialogue(body["choices"][0]["message"]["content"], request_id,
                              input_tokens=int(usage.get("prompt_tokens") or 0),
                              output_tokens=int(usage.get("completion_tokens") or 0),
                              provider=self.model)
