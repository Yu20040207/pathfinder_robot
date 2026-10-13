# 语音评测方法

在机器人实际安装麦克风和扬声器后，至少录制 100 条获得许可的中文话语，覆盖不同说话人、安静与噪声、近讲与实际工作距离、地点名、停止和普通对话。录音统一为单声道 16 kHz PCM16 WAV。不要把录音提交到项目仓库。

清单 CSV 必需字段：`audio_path,reference_text,expected_intent,scenario`；可选 `terms` 以 `|` 分隔本条应识别的专有名词。`expected_intent` 使用 `STOP`、`QUERY_STATUS`、`GO_TO_NAMED_LOCATION` 或 `NONE`。同一清单分别运行 SenseVoice 与 Paraformer，记录模型目录、机器型号、CPU/GPU、线程数、环境噪声与日期。

运行示例：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
.venv/bin/python scripts/benchmark_voice.py /path/to/manifest.csv \
  --model-dir /path/to/sensevoice-model \
  --locations-json /path/to/locations.json \
  --output /path/to/report.json
```

脚本报告字符错误率、专有名词召回率（提供 `terms` 时）、离线意图准确率、停止漏检/误触发和 ASR 推理延迟 P50/P95。总体响应延迟另需从话语结束时间到确认或首句播报时间测量；单独的 ASR 耗时不能代替端到端指标。云端模型的每千次对话费用必须使用真实账单或调用 token 用量及稳定单价，不把免费额度计入长期估算。若样本少于 100 条，结果只用于开发排错，不作为验收结论。

当前无现场语料和确定的 Jetson 型号，因此速度、识别率、麦克风连续运行 2 小时、断网恢复与视觉任务并发均待实测。
