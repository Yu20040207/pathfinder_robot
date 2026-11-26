# 语音唤醒定向 + 走向用户 —— 方案记录（未实现）

> 状态：**提案 / 未实现**，2026-11-26 记录。硬件未采购。
> 本文只记录设计与已验证事实，**不代表任何功能已可用**。
> 现有 `fwr_voice` 的完成度见 §2。

## 1. 用户需求原文

> 「加上一个声源识别的麦克风，然后唤醒之后，首先自动让机器人正对着自己，然后进行答话」

> 「就是为了有时候在机器人的后面唤醒，然后机器人能转过来，用摄像头看到我之后在进行对准，
> 然后说过来再过到面前」

即完整流程：

```
在后背唤醒 → DOA 粗定向 → 转身 → 摄像头确认/精对准 → 答话
   → 用户说"过来" → 走到用户面前停下
```

## 2. 现状：`fwr_voice` 只到「模拟闭环」

`ros2_ws/src/fwr_voice/`（约 1429 行）已完成并可构建，但：

| 项 | 开发机 | 主控（`fwr-n305.local`） |
| --- | --- | --- |
| 包是否安装 | ✅ | ✅ `install/fwr_voice/lib/fwr_voice/{voice_node,voice_demo_executor}` |
| systemd 单元 | ❌ 无 | ❌ **无**（`grep -i voice` 为空），开机不自启 |
| ASR/TTS 模型 | ❌ 无 | ❌ 无（`models/` 只有 `yolo11n.onnx` + face 两个） |
| `FWR_LLM_API_KEY` | ❌ 未设 | ❌ 未设 |
| launch 文件 | ❌ 包内无 `launch/` | 同 |
| 运行中进程 | ❌ | ❌ |
| 麦克风 | ✅ card1 ALC256 | ✅ card0 ALC269VC + card1 5MP USB Camera |
| 扬声器 | ✅ card1 ALC256 Analog | ✅ card0 ALC269VC Analog |

**执行端是模拟的**：`demo_executor.py` 只往 `/demo/cmd_vel` 发速度，
唯一订阅者是 `fwr_navigation/fwr_navigation/voice_chassis_demo.py`（RViz 运动学底盘）。
所以：

- 说「停止」→ 只回一句「已请求停止」，**车不会停**
- 说「去大厅」→ 只 `time.sleep` 模拟移动 2 秒，**不接 Nav2**
- 说「过来」→ **词表里没有这个任务类型**

`docs/superpowers/plans/2026-10-09-voice-interaction.md` 的目标原文就写着
「交付…**模拟** ROS 2 任务执行闭环」，所以现状符合当时计划，只是计划只做到一半。

### 现有可复用资产

| 文件 | 内容 |
| --- | --- |
| `intent.py` | 离线指令：停止/取消/停下来、机器人状态、前进后退左右转、前进N米、去/前往/到X |
| `gate.py` | 授权门：白名单任务类型 + 地点校验 + `SIM_JOG` 仅模拟模式 |
| `dialogue.py` | 云端严格 JSON 解析（`{reply, task}`），任务类型白名单 |
| `contracts.py` | `TASK_TYPES = {STOP, QUERY_STATUS, GO_TO_NAMED_LOCATION, SIM_JOG}` |
| `wake.py` | 唤醒词「小明同学」，30 s 超时 |
| `asr.py` / `tts.py` | sherpa-onnx SenseVoice / VITS（已装 `sherpa_onnx 1.13.8`） |
| `audio.py` | `sounddevice` 优先，**缺失时回退 ALSA `arecord`/`aplay`** |
| `node.py` | `/robot/voice/transcript`、`/robot/voice/response`、`/robot/voice/text_input`、Action `/robot/task/execute` |

### 已存在的房间导航链路（`GO_TO_NAMED_LOCATION` 的真实去处）

`fwr_bringup/fwr_bringup/room_labels.py`（launch 里已起，`real_navigation.launch.py:197`）：

- 订阅 **`/room_goal`**（`std_msgs/String`，房间名）
- 直接发 Nav2 **`navigate_to_pose`**
- `rooms_file` 默认 `share/fwr_bringup/maps/rooms.yaml`

⚠️ 但 **`rooms.yaml` 在开发机和主控上都不存在**（两边 `cat` 均 No such file）→
节点加载 0 个房间，`/room_goal` 只会打 `unknown room`。

⚠️ `find_room()` 是**精确字符串相等**，无别名、无中文数字转换：

```python
if room['name'] == name:   # room_labels_core.py
```

→ rooms.yaml 写 `2号房间`，说「**二**号房间」匹配失败；写 `书房`，说「书房**里**」也失败。

⚠️ `room_labels.py:83` 自己就注明：
`motion gate and arm must already be enabled by the panel`
—— 语音**不能自己开门**。

## 3. 硬件决策：ReSpeaker XVF3000（用户指定，未采购）

选它的理由成立（[Seeed 规格](https://wiki.seeedstudio.com/es/respeaker_mic_array_v3.0/)）：

- **360° DOA**、波束成形、VAD、降噪、AEC 全片上完成
- UAC 1.0 免驱，Linux 直接当声卡
- DOA 通过 **USB 控制传输**读 `DOAANGLE`（0–359，只读），官方 `usb_4_mic_array/tuning.py` 可读

**排除 I2S 类阵列**（树莓派 HAT 如 ReSpeaker 2-Mics Pi HAT）：
主控是 x86 迷你主机（i5-13500H），没有 I2S 排针，只能用 USB 声卡形态。

### 读 DOA 的两条路

| 方式 | 说明 |
| --- | --- |
| 官方 `tuning.py` | Seeed `usb_4_mic_array` 仓库，依赖 `pyusb` |
| `micring`（PyPI） | 跨平台封装 DOA/VAD/调参；**第三方包，装前先审源码** |

两条都需要 **libusb 访问权限**（udev 规则），否则 `Permission denied`。

## 4. 两个必须先验的物理风险

### 4.1 平面 4 麦阵列的方向模糊（最大风险）

4 只麦排成正方形，基线在 0°/90°/45°/135°，**阵列有 90° 旋转对称性**。
理论上有 θ 与 θ+90° 相位模式重合的可能；实际能否分辨依赖波束成形和频率差异，
**必须实测，不能信规格书**。

这个风险正打用户的核心场景（**在机器人后面唤醒**）。

Seeed FAQ 另有一条：不要把 DOA 角度当设备唯一标识，且
**「Orientation depends on build configuration」** → 0° 朝哪由固件决定，装机必须现场标定。

→ **先做静态标定，再写一行代码**（见 §7 P0）。

### 4.2 机器人自己的喇叭带偏 DOA

答话时 TTS 在播，DOA 会指向机器人自己的喇叭（房间里最强声源）。

XVF3000 有 AEC，但**只有 far-end 参考信号真的喂进芯片才生效**。
当前 TTS 从主机声卡出，阵列不知道有这个声音 → 等于 AEC 没开。

第一版做法：**播报期间冻结/丢弃 DOA**（复用 `node.py` 已有的 `EchoGuard` 思路）。
真正 AEC 需要把 TTS 回灌给阵列作参考，属额外工作，暂不做。

## 5. 架构设计（三段，职责分开）

```
① 唤醒 + 定向
     音频流唤醒词 + VAD 有效窗口内 DOA 中位数
        ↓
② 粗转 → 视觉精对准
     IMU 闭环原地转；摄像头确认人；steer() 微调对中
        ↓
③ 答话 → "过来"
     对话；person_follower 接管走向用户
```

### 5.1 转向必须 IMU 闭环，DOA 只做粗定向

DOA 精度通常 ±10~20°，只够转到大方向。

**项目自己的实测数据**（`docs/hardware/real-nav2-acceptance.md:33`）：

> 0.3 秒对应 IMU 约 **+3.68° / −4.16°**，0.5 秒对应 **+18.02° / −23.63°**

左右不对称、不线性 → **开环「转 N 秒」不可用**，时间越长误差越大。

```
DOA θ → 目标偏航 → IMU 闭环 PD 原地转 → 停稳
   → DOA 回读复核（应回到 0°±10°）→ 摄像头找人 → 视觉微调
```

**「转完 DOA 应归零」是天然自检**：转角对了 DOA 会回到 0°；没归零说明转角错或阵列模糊。

可复用现成件：

- `person_follower` 的 `steer()` PD + `OscGuard` 防摆动 + `limit_pair()` 限速
- `fwr_imu` 的 yaw 作闭环反馈（`yaw_scale 1.019`）
- 执行走 `/cmd_vel_manual`，与跟随同通道，天然受 `motion_enabled` + `manual_enabled` 双门控

### 5.2 「过来」是新任务，不是现成能力

`person_follower` 能朝框里的人走，但那是**跟随**语义，不是「走到我面前停下」：

- 它框高 85%（`stop_box_h`）才停 —— 那是「别撞上」的距离，不是「到我面前」
- `manual_max_speed_mps 0.30` 上限 → **跟不上走路的人**
  （要提到 0.5~0.6，属底盘安全参数，**须用户批准**）
- `TASK_TYPES` 和云端白名单里都没有这一类

### 5.3 接口设计（建议最小改动）

| 新增/改动 | 内容 |
| --- | --- |
| 新节点 `voice_doa` | 读 `DOAANGLE` + VAD，发 `/robot/voice/doa`（角度 + 置信度） |
| 新节点 `voice_align` | 消费 DOA + `/scan_merged`（避障）+ `/odom`（IMU），发 `/cmd_vel_manual` |
| `contracts.py` | 加 `COME_TO_ME` |
| `intent.py` | 加「过来 / 到我这儿 / 过来一下」 |
| 执行端 | 替换/旁路 `demo_executor`：`GO_TO_NAMED_LOCATION` → `/room_goal`，`STOP` → 真停车 |

### 5.4 转向阶段的时序约束

**转 → 停稳 → 再收声 → 再答话**。不要边转边识别：
电机噪声 + 移动声源 → DOA 抖动大。

## 6. 安全与授权（**未定，需用户决策**）

「唤醒就转身」= 一说话车就动。但项目现有规则：

- `motion_enabled` / `manual_enabled` **默认 false**
- 节点**不允许自己开门**（见 `AGENTS.md`「代理不得替用户开门」）
- 面板点「开始导航」才开门

三种落法：

| 方案 | 语义 | 代价 |
| --- | --- | --- |
| A | 操作员先用面板开门，语音转动只是「已授权状态下的微调」 | 最安全，每次要点面板 |
| B | 唤醒后语音确认「需要我转向吗？」，说「好」才动 | 多一句交互 |
| C | 只在「交互模式」内允许转（模式由面板开） | ≈A 的模式化版本 |

**建议 A+C**：语音永远不新增「开门」能力。

⚠️ 另需一并解决：**说「停止」现在也不能真停车**（`demo_executor` 只回一句话）。
建议把它排在所有语音运动功能之前。

## 7. 分阶段计划

| 阶段 | 内容 | 工作量 | 风险 |
| --- | --- | --- | --- |
| **P0** | 静态标定 DOA：0/90/180/270° × 1/3/5 m，看有无 90°/前后模糊 | 半天 | 零（只读） |
| **P1** | 唤醒 → DOA → IMU 闭环转向 → 摄像头对中 | 1–2 天 | 中（人说话时车会动） |
| **P2** | 「过来」（复用 follower + 新停止距离） | 2–3 天 | **高**（人车同场移动） |
| **P-1** | **STOP 真能停车** + `rooms.yaml` + 语音接 `/room_goal` | 半天~1 天 | 低 |

**P0 是 go/no-go 关口**：若 180° 与 0° 分不清，整个 DOA 方案作废，
退化成「转一圈用摄像头找人」。

## 8. 开发机先行开发（当前阶段做法）

**可以，而且这正是原计划设计意图**（「在开发电脑上交付…并保留迁移接口」）。

### 8.1 硬件现状（2026-11-26 实测）

开发机：

```
card 0: NVidia [HDA NVidia]      —— 只有 HDMI（3/7/8/9），不能当麦克风/喇叭
card 1: PCH [HDA Intel PCH]      —— ALC256 Analog，device 0 同时支持采集与播放 ✅
```

`arecord -D default -f S16_LE -r 16000 -c 1 -d 1` 实测成功产出 32044 B（≈1 s/16 kHz/mono）。
`aplay -L` 显示 `default` 走 PulseAudio 服务。

⚠️ `sounddevice` 不可用（**PortAudio 缺失**）→ 代码自动回退 `AlsaMicrophoneInput`（`arecord`）
与 `_AlsaPcmSink`（`aplay`）。**这条回退路径必须保持可用**，别只依赖 sounddevice。

### 8.2 能在开发机做什么

| 可做 | 不可做（必须等真机/阵列） |
| --- | --- |
| ASR / TTS / 唤醒词调通 | DOA 相关一切 |
| `intent.py` 指令词表扩充 | 远场唤醒距离调参 |
| 对话路由 + 严格 JSON 校验 | AEC / 回声阈值标定 |
| ROS 接口与 Action 链路 | 转向闭环的实机验证 |
| 房间名解析（`rooms.yaml`） | 底盘限速相关调参 |
| 单元测试 | 主控音频设备选型 |

### 8.3 开发机 ≠ 主控 的四个差异（别把调参结果直接搬过去）

1. **设备名不同**：开发机 `card1/ALC256`，主控 `card0/ALC269VC` + `card1/5MP USB Camera`
2. **近场 vs 远场**：开发机麦是近场 → 唤醒词距离、增益、EchoGuard 阈值**必须在主控重测**
3. **回声耦合更强**：开发机麦和喇叭同机壳，AEC 问题比车上更严重 → 阈值不能在此标定
4. **无 DOA**：整个「转过来」的能力在开发机无法验证

### 8.4 部署到主控的成本

两边都是 **x86_64 / Ubuntu 22.04 / Python 3.10** → **无交叉编译**，成本很低：

1. 模型文件拷过去（**不进仓库**，原计划明确要求）
2. `colcon build --packages-select fwr_voice --symlink-install`
3. 加 systemd 单元（**必须带 `Environment=HOME=/home/manbaout`**，
   缺 HOME 会 `failed to configure logging` 后 abort，2026-11-24 踩过）
4. 显式指定音频设备（见 §8.5）

### 8.5 已发现的代码缺口：音频设备无法配置

`node.py:284` 是 `microphone = MicrophoneInput()`，**没有 device 参数**，
`node.py` 也**没有声明任何麦克风/扬声器设备参数**。

后果：只能用 ALSA `default`。开发机上 `default` 采集可用，
但**播放可能落到 HDMI（无声）**；主控上 `default` 更不可控。

→ **开发第一阶段就要补的两个 ROS 参数**：`microphone_device`、`speaker_device`
（值形如 `plughw:1,0` / `hw:1,0`，可留空走 default）。

## 9. 待确认事项

| # | 问题 | 影响 |
| --- | --- | --- |
| 1 | 授权模型选 A / B / C？ | 决定能否碰 `/cmd_vel_manual` |
| 2 | 麦克风阵列预算 + 安装位置（车顶正中/车头） | 选型与 0° 参考方向 |
| 3 | 车上是否已装扬声器？装在哪？ | AEC 与回声策略 |
| 4 | 「过来」的停止距离定义（离人多远算「面前」） | P2 验收标准 |
| 5 | `rooms.yaml` 的房间命名（要口语、不与中文数字冲突） | 语音「去X」能否匹配 |

## 10. 相关文件

- `ros2_ws/src/fwr_voice/` —— 语音包（模拟闭环）
- `ros2_ws/src/fwr_bringup/fwr_bringup/room_labels.py` —— `/room_goal` → Nav2
- `ros2_ws/src/fwr_bringup/fwr_bringup/room_labels_core.py` —— `load_rooms` / `find_room`
- `ros2_ws/src/fwr_bringup/fwr_bringup/person_follower.py` + `follow_core.py` —— 可复用的转向/避障
- `ros2_ws/src/fwr_navigation/fwr_navigation/voice_chassis_demo.py` —— `/demo/cmd_vel` 唯一订阅者
- `docs/superpowers/plans/2026-10-09-voice-interaction.md` —— 原计划（模拟闭环）
- `docs/superpowers/specs/2026-10-09-voice-interaction-design.md` —— 设计稿
- `docs/voice/benchmark-protocol.md` —— ASR 基准协议
- `docs/hardware/real-nav2-acceptance.md:33` —— IMU 转向实测数据
