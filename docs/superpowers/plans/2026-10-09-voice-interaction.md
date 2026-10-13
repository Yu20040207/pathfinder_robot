# Voice Interaction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在开发电脑上交付中文离线基本指令、联网自然对话、语音输入输出和模拟 ROS 2 任务执行闭环，并保留 Jetson 迁移接口。

**Architecture:** `fwr_voice` 的纯 Python 核心处理音频、识别、路由、对话和播报；ROS 2 节点只负责消息与 Action 适配。所有动作经命名任务白名单和状态门控，模拟执行器是本阶段唯一执行端。

**Tech Stack:** Ubuntu、ROS 2 Humble、Python 3、`rclpy`、`rosidl`、pytest；ASR 候选 FunASR SenseVoice Small/Paraformer，TTS 候选 sherpa-onnx 中文模型；云端文本模型通过 HTTP 适配接口。

**Spec:** `docs/superpowers/specs/2026-10-09-voice-interaction-design.md`

## Global Constraints

- 当前导航只发布规划路径；语音代码绝不发布 `/cmd_vel`，也不直接调用电机。
- 初版动作只有 `STOP`、`QUERY_STATUS`、`GO_TO_NAMED_LOCATION`；地点只取配置中的 ID。
- 断网时三类基本指令仍可工作；开放对话允许降级提示。
- 模型文件、API 密钥和默认录音不得写入仓库；默认日志不存录音与完整对话。
- 电脑端先走模拟任务；Jetson 型号和实机任务执行器未确定。
- 本目录目前没有 `.git`；执行期间每任务运行验证并记录结果，只有用户另行建立 Git 仓库后才做提交。

## Review Focus

- “停止”与普通对话同时到达：停止先取消当前任务，不等待云端返回；Task 2 和 Task 5 测试。
- 云端返回伪造坐标、速度或 ROS 话题：拒绝动作；Task 3 测试。
- 地点名称相似或不在白名单：不选择最接近项自动移动；Task 2 测试。
- 音频设备消失或 ASR 返回空文本：节点仍可报告错误并处理下次请求；Task 4 测试。
- 云端超时后迟到的响应：不得执行过期动作；Task 3 和 Task 5 测试。

## File Map

| 文件 | 职责 |
| --- | --- |
| `ros2_ws/src/fwr_interfaces/msg/VoiceTranscript.msg`, `VoiceResponse.msg` | 可观察的识别与回答结果 |
| `ros2_ws/src/fwr_interfaces/action/ExecuteRobotTask.action` | 有限任务及执行状态 |
| `ros2_ws/src/fwr_voice/fwr_voice/contracts.py` | 纯 Python 内部数据类型和任务常量 |
| `.../intent.py`, `gate.py` | 本地指令判定、白名单与状态门控 |
| `.../dialogue.py`, `providers.py` | 对话协议、HTTP 适配与费用记录 |
| `.../audio.py`, `asr.py`, `tts.py` | 音频设备和模型适配 |
| `.../pipeline.py`, `node.py`, `demo_executor.py` | 流水线、ROS 2 节点、模拟 Action 服务 |
| `ros2_ws/src/fwr_bringup/config/voice.yaml`, `launch/voice_demo.launch.py` | 电脑端参数和启动 |
| `tests/test_voice_*.py`, `scripts/benchmark_voice.py` | 逻辑回归与测量 |

### Task 1: 固定 ROS 2 契约与可安装包

**Files:** Create `ros2_ws/src/fwr_interfaces/msg/VoiceTranscript.msg`, `VoiceResponse.msg`, `action/ExecuteRobotTask.action`; modify `ros2_ws/src/fwr_interfaces/CMakeLists.txt`, `package.xml`; create `ros2_ws/src/fwr_voice/{package.xml,setup.py,setup.cfg,resource/fwr_voice,fwr_voice/__init__.py,fwr_voice/contracts.py}`; test `tests/test_voice_contract.py`.

**Interfaces:** `VoiceTranscript` = `std_msgs/Header header`, `string text`, `string provider`, `float32 latency_ms`; `VoiceResponse` = `Header header`, `string text`, `string status`, `string error_code`. `ExecuteRobotTask` goal = `string request_id`, `string task_type`, `string target_id`; result = `bool success`, `string status`, `string message`; feedback = `string status`, `string message`. `contracts.py` 定义 `TaskRequest(request_id: str, task_type: str, target_id: str = "")` 和 `TaskDecision(kind: str, request: TaskRequest | None, reply: str)`；`kind` 只用 `task`、`reply`、`clarify`。

- [ ] 写接口测试，断言消息与 Action 定义字段、允许任务常量和包元数据。
- [ ] 运行 `.venv/bin/python -m pytest tests/test_voice_contract.py -q`，确认因缺文件失败。
- [ ] 添加接口定义与 `fwr_voice` 最小包，更新 `rosidl_generate_interfaces` 和依赖。
- [ ] 重跑接口测试，并从已 source 的 ROS 2 终端执行 `cd ros2_ws && colcon build --packages-select fwr_interfaces fwr_voice --symlink-install`，确认成功。
- [ ] 如仓库已建立，提交接口与包骨架；否则记录未提交原因。

### Task 2: 离线指令判定与安全门控

**Files:** Create `ros2_ws/src/fwr_voice/fwr_voice/{intent.py,gate.py}`; test `tests/test_voice_intent.py`, `tests/test_voice_gate.py`.

**Interfaces:** `parse_local(text: str, locations: dict[str, tuple[str, ...]]) -> TaskDecision`；`GateState(busy: bool, executor_ready: bool, simulated: bool)`；`authorize(request: TaskRequest, state: GateState, allowed_location_ids: set[str]) -> TaskDecision`。本地停止词如“停止”“取消任务”总是映射 `STOP`；“去<别名>”仅精确命中唯一地点；“机器人状态”映射 `QUERY_STATUS`。空文本与歧义返回 `clarify`。

- [ ] 写测试：停止优先、状态查询、地点唯一/重名/未知、空文本，以及忙碌、执行器缺失、非法任务和模拟模式播报。
- [ ] 运行上述两个测试文件，确认缺实现导致失败。
- [ ] 实现文本标准化、确定性解析和门控；不引入模糊匹配或低层运动参数。
- [ ] 重跑两个测试文件，确认全部通过。
- [ ] 如有 Git，提交本地指令与门控。

### Task 3: 云端对话适配与严格结构化输出

**Files:** Create `ros2_ws/src/fwr_voice/fwr_voice/{dialogue.py,providers.py}`; test `tests/test_voice_dialogue.py`.

**Interfaces:** `DialogueProvider.respond(text: str, allowed_tasks: tuple[str, ...], status_summary: str, request_id: str) -> DialogueResult`；`DialogueResult(reply: str, task: TaskRequest | None, input_tokens: int, output_tokens: int, provider: str)`；`route_cloud(result: DialogueResult, active_request_id: str, state: GateState, allowed_location_ids: set[str]) -> TaskDecision`。HTTP 后端参数为 `base_url`、`model`、`api_key_env`、`timeout_s=3.0`、`max_output_tokens=256`；仅接受一个符合 schema 的 JSON 对象。

- [ ] 写测试：普通回答、合法结构化任务、非 JSON、额外字段、伪造坐标/速度/话题、未知地点、超时、迟到 request ID；断言只有合法任务进入 `authorize`。
- [ ] 运行 `tests/test_voice_dialogue.py`，确认失败。
- [ ] 实现供应商无关协议、一个 OpenAI 兼容 HTTP 文本后端、严格解析与用量记录；密钥只从环境读取，绝不写日志。
- [ ] 重跑测试，使用假 HTTP 服务覆盖成功与超时，不调用付费 API。
- [ ] 如有 Git，提交对话适配。

### Task 4: 本地音频、ASR 与 TTS 适配

**Files:** Create `ros2_ws/src/fwr_voice/fwr_voice/{audio.py,asr.py,tts.py}`; modify `requirements-runtime.txt`; test `tests/test_voice_audio.py`; create `scripts/benchmark_voice.py`.

**Interfaces:** `AudioInput.record_once(max_duration_s: float) -> AudioClip`；`AudioClip(pcm16: bytes, sample_rate_hz: int, ended_monotonic_s: float)`；`AsrProvider.transcribe(clip: AudioClip) -> Transcript(text: str, provider: str, latency_ms: float)`；`TtsProvider.speak(text: str) -> None`。实现文件/WAV 注入和设备采集两种输入，SenseVoice Small 本地适配为默认候选，Paraformer 为可选候选；TTS 适配一个中文本地模型。模型初始化延迟与单句延迟分别记录。

- [ ] 写测试：16 kHz 单声道 PCM、最大录音时长、设备断开、空识别、TTS 错误、播报期间停止监听仍可中断播报；假模型测试接口，真实模型使用受控短 WAV 做冒烟测试。
- [ ] 运行 `tests/test_voice_audio.py`，确认失败。
- [ ] 实现适配层和可配置模型路径；设备采集失败返回明确错误并允许下一次重试。TTS 播放期间暂停普通对话采音，但保留可中断播报的本地停止监听；音频设备不支持全双工时明确报告该限制，物理急停仍由硬件链路负责。
- [ ] 重跑单测和受控 WAV 冒烟测试；记录两种 ASR 在同一文件上的文本、耗时和峰值内存，不能把冒烟结果当作准确率结论。
- [ ] 如有 Git，提交音频链路。

### Task 5: 流水线、ROS 2 节点与模拟任务

**Files:** Create `ros2_ws/src/fwr_voice/fwr_voice/{pipeline.py,node.py,demo_executor.py}`; modify `ros2_ws/src/fwr_voice/setup.py`, `ros2_ws/src/fwr_bringup/package.xml`; create `ros2_ws/src/fwr_bringup/config/voice.yaml`, `launch/voice_demo.launch.py`; test `tests/test_voice_pipeline.py`, `tests/test_voice_ros.py`.

**Interfaces:** `VoicePipeline.handle_text(text: str, request_id: str) -> TaskDecision` 和 `handle_clip(clip: AudioClip, request_id: str) -> TaskDecision`；ROS 节点发布 `/robot/voice/transcript` 与 `/robot/voice/response`，调用 `/robot/task/execute` Action；模拟执行器只接受三类任务并提供取消反馈。配置使用 `mode: simulation`，地点别名表与模型/设备参数在 `voice.yaml`；API 密钥不在 YAML 中。

- [ ] 写测试：本地指令不触发云调用、普通对话不会触发 Action、停止取消进行中任务且先于迟到对话响应、模拟移动有明确提示、无执行器拒绝、话题中无 `/cmd_vel`。
- [ ] 运行两个测试文件，确认失败。
- [ ] 实现流水线、ROS 节点、模拟 Action 服务与 launch；提供文本输入模式和音频输入模式，默认模拟模式。
- [ ] 重跑测试及 `colcon build --packages-select fwr_interfaces fwr_voice fwr_bringup --symlink-install`；启动 `voice_demo.launch.py` 验证状态与取消。
- [ ] 如有 Git，提交端到端演示。

### Task 6: 操作说明、成本与质量基线

**Files:** Modify `README.md`; create `docs/voice/benchmark-protocol.md`; modify `scripts/benchmark_voice.py`; test `tests/test_voice_benchmark.py`.

**Interfaces:** 基准脚本输入为包含 `audio_path,reference_text,expected_intent,scenario` 的 CSV，输出逐条 JSONL 与汇总 JSON：CER、专有名词召回、意图准确率、停止漏检/误触发、P50/P95 延迟、模型与硬件信息；云端费用按实际用量和配置的单价计算，不把免费额度算作长期价格。

- [ ] 写测试：中文 CER 计算、空参考文本、百分位数、小样本、缺失音频、费用汇总。
- [ ] 运行 `tests/test_voice_benchmark.py`，确认失败。
- [ ] 完成基准脚本与 README 启动、配置、断网、密钥和 Jetson 迁移说明；记录现场至少 100 条话语的采集格式，未采集前不宣称达标。
- [ ] 运行 `.venv/bin/python -m pytest tests -q`、ROS 2 构建和端到端模拟演示；在真实设备可用时再测麦克风连续运行 2 小时与断网恢复。报告已通过和未验证项目。
- [ ] 如有 Git，提交文档与基准工具。

## Completion Gate

源码可在开发电脑以文本/WAV 注入和模拟执行器形成完整闭环；本地三类指令无需云端；云端回答与任务 JSON 均有严格边界；`/cmd_vel` 不由语音包发布。真实性能、现场识别率、Jetson 并发和实机任务只有在对应设备与标注数据齐备后才可验收。
