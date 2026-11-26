# 语音开发接入（开发机，2026-11-26 实测）

> 本文只写**已验证**的事实和可复制的命令。方案设计见
> `docs/hardware/voice-wake-align-proposal.md`。
> 所有数字都是 2026-11-26 在开发机（`Yu`，x86_64 / Ubuntu 22.04）上跑出来的。

## 1. 一句话现状

**开发机上「唤醒 → 离线识别 → 回复 → 合成播报」已经打通**，用的是普通 USB 声卡 + 本地
sherpa-onnx 模型，不联网、不需要 API key。

**还没做的**：真实麦克风远场唤醒距离、声源定向（DOA）、接真车执行端。

## 2. 声卡：开发机上有三张，别选错

```
card 0  NVidia     只有 HDMI（3/7/8/9）        ← 播不出、录不到，别用
card 1  PCH ALC256 Analog                      ← 机身内置，可播可录
card 2  USB Audio Device (Generalplus 1b3f:2008) ← 外接声卡/音箱，可播可录
```

- 外接 USB 声卡插上后**编号可能是 2，也可能变**（取决于插入顺序和是否重启）。
  每次插拔后重新 `aplay -l | grep ^card` 确认。
- 名字形式更稳：`plughw:CARD=Device,DEV=0`（USB 声卡）、`plughw:CARD=PCH,DEV=0`（内置）。
- `/dev/snd/by-id/` 只有 control 设备有稳定链接，**PCM 没有**，所以 `plughw:2,0` 只能按编号写。

### 快速自检两条

```bash
# 播（能听到声音才算通）
aplay -q -D plughw:2,0 /tmp/tts_probe.wav

# 录 3 秒看有没有数据（RMS 一两百 = 环境噪声正常；0 = 没接到信号）
arecord -q -D plughw:2,0 -t raw -f S16_LE -r 16000 -c 1 -d 3 /tmp/m.raw
python3 -c "import audioop;d=open('/tmp/m.raw','rb').read();print(len(d), audioop.rms(d,2))"
```

## 3. 模型：已下到 `models/voice/`（`models/` 已被 .gitignore 忽略）

| 目录 | 内容 | 大小 |
| --- | --- | --- |
| `models/voice/sensevoice/` | `model.int8.onnx` + `tokens.txt` | 239 MB + 316 KB |
| `models/voice/vits/` | `model.onnx` + `tokens.txt` + `lexicon.txt` | 121 MB + 331 B + 2.4 MB |
| `models/voice/test_wavs/zh.wav` | 官方中文测试音频（16 kHz mono, 5.6 s） | 179 KB |

来源（走 `hf-mirror.com`，GitHub 在本机不通）：

- SenseVoice：`csukuangfj/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17`
- VITS：`csukuangfj/vits-zh-hf-fanchen-C`（仓库里文件名是 `vits-zh-hf-fanchen-C.onnx`，
  本地存成 `model.onnx`，因为代码按 `model.onnx` 找）

VITS 这个模型 **187 个音色**（`tts_speaker_id` 可选 0~186）、输出 16 kHz。

## 4. 实测性能

| 项 | 数值 |
| --- | --- |
| SenseVoice 加载 | ~1 s（首次），常驻后单句 **67~70 ms**（5 s 音频） |
| VITS 加载 | 392 ms |
| VITS 合成 | 3.48 s 音频用 **1606 ms**（RTF ≈ 0.46） |
| 唤醒闭环（合成「小明同学」→ 认出 → 回「我在，请说。」） | 识别 **67 ms**，之后立即播报 |

## 5. 怎么跑（三条路，从简到全）

### 5.1 只验 ASR（不碰 ROS）

```bash
cd ~/Company_Project/Four-wheel-robot
PYTHONPATH=ros2_ws/src/fwr_voice .venv/bin/python - <<'EOF'
from fwr_voice.audio import WavFileInput
from fwr_voice.asr import SherpaSenseVoice
clip = WavFileInput('models/voice/test_wavs/zh.wav').record_once(15)
t = SherpaSenseVoice('models/voice/sensevoice').transcribe(clip)
print(t.text, f"{t.latency_ms:.0f} ms")
EOF
```

实测输出：`开饭时间早上9点至下午5点。 167 ms`

### 5.2 只验 TTS（不碰 ROS、不接麦克风）

```bash
PYTHONPATH=ros2_ws/src/fwr_voice .venv/bin/python - <<'EOF'
from fwr_voice.tts import SherpaVitsTts
SherpaVitsTts('models/voice/vits', device='plughw:2,0').speak('我已经转向你了，请说指令。')
EOF
```

### 5.3 起整个节点（推荐用 launch）

```bash
cd ~/Company_Project/Four-wheel-robot
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# ① 不开麦，用 WAV 走一遍唤醒闭环（最稳的冒烟）
ros2 launch fwr_voice voice.launch.py \
  input_wav:=/tmp/wake.wav speaker_device:=plughw:2,0

# ② 开麦（真说话）
ros2 launch fwr_voice voice.launch.py \
  microphone_enabled:=true \
  microphone_device:=plughw:2,0 speaker_device:=plughw:2,0
```

期望日志：

```
[voice_node]: 识别[sensevoice, 67 ms]: 小明同学。
[voice_node]: 输出[awake]: 我在，请说。
```

> `/tmp/wake.wav` 是自己合成的唤醒词音频，生成方法见 §6。

## 6. 生成一个「唤醒词」WAV（用于不开麦冒烟）

```bash
PYTHONPATH=ros2_ws/src/fwr_voice .venv/bin/python - <<'EOF'
import wave, numpy as np
from fwr_voice.tts import SherpaVitsTts
audio = SherpaVitsTts('models/voice/vits').engine.generate("小明同学")
pcm = (np.clip(audio.samples, -1, 1) * 32767).astype("<i2")
with wave.open('/tmp/wake.wav', 'wb') as w:
    w.setnchannels(1); w.setsampwidth(2); w.setframerate(audio.sample_rate)
    w.writeframes(pcm.tobytes())
EOF
```

反查过：这句合成语音喂回 SenseVoice 得到 `小明同学。`，所以能可靠触发唤醒门。

`input_wav` 要求 **mono / 16 kHz / PCM16 / ≤15 s**，否则 `WavFileInput` 直接抛异常。

## 7. 两个必须知道的坑

### 7.1 `ros2 launch` 默认用 `/usr/bin/python3`，看不到 `sherpa_onnx`

`sherpa_onnx` 只装在项目 `.venv/` 里。用系统 python 起节点会：

```
ModuleNotFoundError: No module named 'sherpa_onnx'
```

`voice.launch.py` 已经处理：它把节点 `prefix` 成仓库的 `./.venv/bin/python`
（和 systemd 单元 `./.venv/bin/python ...` 的写法一致）。可用 `FWR_VOICE_PYTHON` 覆盖。
副作用：launch 里节点显示成 `python-1` 而不是 `voice_node-1`，**但 ROS 节点名仍是
`/voice_node`**，`ros2 node list` 能查到。

### 7.2 设备名写错会**立刻报错**，不会静默走默认

实测把 `plughw:1,0` 误写成 `plhw:1,0`：

```
[voice_node]: TTS failure: RuntimeError
```

这是对的——`play_with_alsa` 的 `aplay` 返回非 0 就抛 `RuntimeError("ALSA playback failed")`。
**听到「没声音但也没报错」时，先查是不是走了 default 而不是你以为的那张卡。**

## 8. 测试怎么跑（pytest 有个环境坑）

**坑**：先 `source /opt/ros/humble/setup.bash` 再跑 pytest，会让 `tests/` 目录**收集到 0 个用例**
（连既有的 `test_follow_core.py` 也一样，与语音改动无关）。ROS 侧注入的
`ament-lint` / `launch-testing` / `colcon-core` 都注册了 pytest hook，逐个 `-p no:` 关掉仍复现。

**可用配方**（关掉第三方插件自动加载，但仍保留 ROS 的 `LD_LIBRARY_PATH`）：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests
```

实测（2026-11-26）：**465 passed, 1 skipped**（跳过的是 `test_cloud_media_relay.py`，缺 fastapi）。

纯 Python 用例也可以完全不 source ROS：

```bash
PYTHONPATH=ros2_ws/src/fwr_voice .venv/bin/python -m pytest -q tests/test_voice_*.py
```

实测：**77 passed**（含两个需要 rclpy 的用例时要用上面的配方）。

## 9. 本轮代码改动

| 文件 | 改动 |
| --- | --- |
| `fwr_voice/node.py` | 新增参数 **`microphone_device`**、**`speaker_device`**（默认空 = ALSA default）；加 `audio_device()` 归一化空串；把设备透传给 `MicrophoneInput` / `SherpaVitsTts` / `DashscopeCosyVoiceTts` |
| `fwr_voice/tts.py` | 新增 `_alsa_command(rate, device)`；`play_with_alsa(..., device=None)`、`_AlsaPcmSink(device=None)`、`DashscopeCosyVoiceTts(..., device=None)`、`SherpaVitsTts.speak` 把 `self.device` 传下去 |
| `fwr_voice/launch/voice.launch.py` | **新增**：模型目录自动推断（`FWR_VOICE_MODELS` → 源码树 → `~/Company_Project/...`）、设备参数、`prefix` 到 venv python、可选 `start_demo_executor` |
| `fwr_voice/setup.py` | `data_files` 加 `share/fwr_voice/launch` |
| `tests/test_voice_audio.py` | 适配新签名（`play_with_alsa` 多一个 `device=`）+ 补 `voice.device = None` |
| `tests/test_voice_tts_speaker.py` | monkeypatch 的 lambda 改成 `*args, **kwargs` |

**没改任何执行端逻辑**：`demo_executor` 仍然只发 `/demo/cmd_vel`，说「停止」仍然不会让车停。

## 9.5 豆包（火山引擎）音色 —— ✅ **已用真实账号打通（2026-11-26）**

豆包音色是**云端服务**，本地模型里没有。**协议已实测验证**，默认仍是本地 VITS，
要豆包音色就显式选 `tts_provider:=doubao`。

### ⚠️ 先搞清楚：豆包语音有三套接口，别混

| | 经典版 | **Seed-TTS（能用的是这个）** | 大模型 WebSocket |
| --- | --- | --- | --- |
| 端点 | `/api/v1/tts` | **`/api/v3/tts/unidirectional`** | `/api/v1/tts/ws_binary` |
| 凭据 | appid + access token | **豆包语音「API Key」（UUID）** 放 `X-Api-Key` | 同上 |
| 资源标识 | body 里 `cluster` | **`X-Api-Resource-Id: seed-tts-2.0`** | 头 |
| 响应 | 单个 JSON | **NDJSON 流**（多块，`code=20000000` 收尾） | 二进制帧 |
| 音色名 | `BV700_streaming` | **`saturn_*_tob`** | — |

**实测踩过的三个坑**（每个错一层，报错还不带区分）：

1. 把 API Key 塞进 `Authorization` 或 body 的 `app.token` →
   `invalid auth token` / `load grant: requested grant not found`；
2. 不带资源头 → `get resource id empty`；用没开通的资源 →
   `requested resource not granted`；
3. 拿 `*_bigtts` 音色喂 2.0 → **HTTP 200** 但
   `code=55000000 resource ID is mismatched with speaker related resource`。

### 第一步：拿凭据

| 项 | 在哪 |
| --- | --- |
| **API Key** | 豆包语音控制台 → 左侧「API Key」→ 点眼睛复制（**UUID 形态，不以 `ark-` 开头**） |
| **资源 ID** | 语音合成 2.0 = `seed-tts-2.0`；3.0 = `seed-tts-3.0`（**需先开通**） |
| **音色 ID** | 控制台「音色库」筛「语音合成 2.0」，或文档 <https://docs.volcengine.com/docs/doubao-voice/tone-list> |

```bash
echo '<API Key>' > ~/.volc_token && chmod 600 ~/.volc_token
export FWR_VOLC_APIKEY=$(cat ~/.volc_token)
```

### 第二步：试听音色（音色名**不能猜**）

```bash
.venv/bin/python tools/volc_voice_audition.py \
  --voices "saturn_zh_female_keainvsheng_tob,saturn_zh_male_shuanglangshaonian_tob" \
  --out models/voice/audition_cloud
```

生成 `models/voice/audition_cloud/index.html`，浏览器逐个试听。
报 `resource ID is mismatched` 就是**这个音色不属于 2.0**。

已验证可用：

| 音色 | 风格 | 实测时长 |
| --- | --- | --- |
| `saturn_zh_female_keainvsheng_tob` | 可爱女生 | 2.71 s |
| `saturn_zh_male_shuanglangshaonian_tob` | 爽朗少年 | 2.15 s |

### 第三步：接进对话（**定版：用这个脚本**）

```bash
./tools/run_voice.sh          # 开麦，说「小明同学」唤醒
./tools/run_voice.sh --wav    # 不开麦冒烟
./tools/run_voice.sh --local  # 退回本地 187 音色
```

脚本默认就带好了 `tts_provider:=doubao` + `tts_seed_voice:=saturn_zh_female_keainvsheng_tob`
+ 开发机的 `mic=plughw:1,0` / `speaker=plughw:2,0`，并自动从 `~/.volc_token` 读 API Key。

等价的长命令（换机器/换声卡时照着改）：

```bash
export FWR_VOLC_APIKEY=$(cat ~/.volc_token)

ros2 launch fwr_voice voice.launch.py microphone_enabled:=true \
  microphone_device:=plughw:1,0 speaker_device:=plughw:2,0 \
  tts_provider:=doubao \
  tts_seed_voice:=saturn_zh_female_keainvsheng_tob
```

> **2026-11-26 定版音色 = `saturn_zh_female_keainvsheng_tob`（可爱女生）**，
> 它已经是 `tts_seed_voice` 的默认值，不传也行。

云端失败会**自动回退本地 VITS**（`FallbackTts`），日志里出现
`豆包 Seed-TTS 不可用，使用本地音色: <原因>`，不会哑掉。

### 相关参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `tts_provider` | `local` | `local` / `doubao` / `aliyun` / `volcengine` |
| `tts_seed_voice` | `saturn_zh_female_keainvsheng_tob` | 音色 ID |
| `tts_seed_resource_id` | `seed-tts-2.0` | 2.0 用这个；开通 3.0 后改 `seed-tts-3.0` |
| `tts_seed_encoding` | `pcm` | 返回编码 |
| `tts_seed_apikey_env` | `FWR_VOLC_APIKEY` | 存 API Key 的环境变量名 |

### 计费

语音合成 2.0 按**字符**计额度（控制台「开通管理」里显示剩余字数）。
一次「你好，我是机器人小助手。」≈ 12 字，测试几十轮也就几百字。


## 9.6 云端对话（方舟）—— 「当前离线」怎么修

默认 `cloud_base_url` 为空时，节点只回**「当前离线，暂时无法进行开放对话。」**。
接上一个 OpenAI 兼容的大模型端点就会变成真对话。

### 2026-11-26 定版配置

| 项 | 值 |
| --- | --- |
| 端点 | `https://ark.cn-beijing.volces.com/api/v3`（火山方舟） |
| 模型 | **`doubao-seed-2-0-lite-260428`** |
| 凭据 | 方舟 **API Key**（`ark-` 开头），放 `~/.ark_key` |
| 关思考 | `FWR_CLOUD_EXTRA_JSON='{"thinking":{"type":"disabled"}}'` |

`./tools/run_voice.sh` 已经全帮你配好，只要 `~/.ark_key` 存在。

### ⚠️ 方舟的三个坑（都实测踩过）

1. **模型列表 ≠ 已开通**：`GET /api/v3/models` 列出 55 个模型，
   但只有 `doubao-seed-2-0-lite-260428` 真能调，其余报
   `404 ModelNotOpen`（要在方舟控制台「开通管理」里逐个开通）。
2. **思考型模型会先吐 2~3 秒思考过程**：`doubao-seed-2-0-lite` 默认输出
   `reasoning_content`，首字 **3060 ms**；
   `enable_thinking: false`（通义那套）**方舟忽略**，仍 2694 ms；
   必须用方舟的 **`thinking: {"type":"disabled"}`** → **首字 0 ms**。
3. **流式响应没有 charset**：`requests` 会按 latin-1 解 → 中文变 `æ`，
   还会因多字节字符被行切分而报 `Unterminated string`。代码已改成自己按 UTF-8 解字节。

### 为什么 JSON 开关走环境变量而不是 launch 参数

```bash
# ❌ 会报错：launch 把右边当 YAML 解析成 dict
ros2 launch ... cloud_extra_json:='{"thinking":{"type":"disabled"}}'

# ✅ 正确
export FWR_CLOUD_EXTRA_JSON='{"thinking":{"type":"disabled"}}'
```

launch 参数只能是标量；这类 JSON 配置统一用环境变量 `FWR_CLOUD_EXTRA_JSON`。

### 换别的大模型

只要是 OpenAI 兼容接口（`<base_url>/chat/completions`、`Authorization: Bearer`）都能用：

```bash
cloud_base_url:=https://ark.cn-beijing.volces.com/api/v3
cloud_model:=<模型ID或 ep- 接入点>
cloud_api_key_env:=FWR_LLM_API_KEY
```

### 已知局限

**模型不知道当前日期时间**，问「今天星期几」会编（实测答"星期三"，
而 2026-11-26 是星期四）。要修得在 system prompt 里注入当前时间——本轮未做。

## 10. 下一步

| # | 事项 | 状态 |
| --- | --- | --- |
| 1 | 开发机声卡 + 模型 + 唤醒闭环 | ✅ 已验证 |
| 2 | 真实麦克风说话测试（`microphone_enabled:=true`） | ⏳ 需要人对着麦说「小明同学」 |
| 3 | `rooms.yaml` + 语音接 `/room_goal` | 未开始 |
| 4 | `STOP` 接真实执行端（真能停车） | 未开始 |
| 5 | 部署到主控（模型拷过去 + systemd 单元） | 未开始 |
| 6 | XVF3000 到货 → P0 静态 DOA 标定 | 未开始（硬件未采购） |
