# 视觉方案（提议稿）

日期：2026-11-21 · 状态：**方案建议，除 HP60C 考勤链路外均未实施**

本文回答三个问题：抓取要不要夹爪端深度相机、机身视频流用什么相机、已有 2D 双雷达时深度相机还有什么用。
实机现状与参数以 `AGENTS.md` §3、排查过程以 `docs/AGENT_LOG.md` 为准。

---

## 1. 现有相机

| 项 | 值 |
| --- | --- |
| 型号 | **HP60C**（`ascamera` / `ascamera_node`，Orbbec 系方案） |
| 话题 | `/robot/camera/color/image_raw`、`/robot/camera/depth/image_raw`（+ `camera_info`） |
| 分辨率 / 帧率 | 640×480 RGB + 640×480 深度，**25 fps**（`config/hp60c.yaml`） |
| 启动 | `launch/vision_attendance.launch.py`（`use_camera_driver:=true`） |
| 当前用途 | **只做刷脸考勤**（`fwr_vision` → `fwr_attendance`），深度通道基本闲置 |

> 结论：**车上已经有一台深度相机**。"要不要上深度"其实是"要不要把已有深度接进避障，以及抓取要不要另加一台短距的"。

---

## 2. 抓取：要深度，不要纯 RGB

| 方案 | 抓取精度 | 成本 | 判断 |
| --- | --- | --- | --- |
| 纯 RGB 单目 | 靠先验尺寸估尺度，误差 2~5 cm | ¥100 | ❌ 最后 2 cm 对不准 |
| **夹爪端短距深度（结构光/ToF）** | 0.3~1 cm @20~50 cm | ¥1,000~3,600 | ✅ 推荐 |
| 用机身 HP60C（eye-to-hand） | 1~3 cm，末端会遮挡目标 | 已有 | ⚠️ 只能粗定位 |

关键理由：抓取需要**米制三维位姿**。单目只能靠"我知道这个杯子 8 cm 宽"的先验，换物体就失效；深度直接给点云，`RANSAC 平面分割 → 物体聚类 → 位姿`，误差可控。

机身相机的两个硬伤：① 距离 0.8~1.2 m，深度误差随距离平方放大；② 视角从下往上看桌面，夹爪自己会挡住目标。夹爪端（eye-in-hand）离物体 15~30 cm，正落在短距深度相机的最佳区间。

### 候选型号

| 型号 | 有效距离 | 价格区间 | 备注 |
| --- | --- | --- | --- |
| **RealSense D405** | 7~50 cm | ¥2,800~3,600 | 夹爪端事实标准，体积小、短距精度最好 |
| OAK-D-Lite | 20~100 cm | ¥1,200 | 相机内可跑推理，省主控 CPU |
| Orbbec Gemini 2L / Astra Pro Plus | 30~200 cm | ¥1,000~1,600 | 便宜、ROS 2 驱动成熟（与 HP60C 同厂） |

### 三个必须做的功课

1. **手眼标定**（eye-in-hand）：`easy_handeye2` + MoveIt2；误差 >5 mm 基本抓空。
2. **时间同步**：深度帧与机械臂/底盘位姿用 `message_filters::ApproximateTime` 或硬件触发，否则运动中点云拖影。
3. **环境限制**：结构光在强阳光、黑色哑光表面会失效（室内可接受）。

---

## 3. 机身视频流：RGB 广角足够，不要传深度

| 用途 | 建议 |
| --- | --- |
| 远程观看/接管 | **120° 广角 RGB，720p@15fps** |
| 深度点云 | 只给车端避障/抓取后端用，**不往 4G 上传** |

HP60C 水平视场约 60°，开车时看不到两侧，不适合做直播主视角；建议另加一只 ¥100~200 的 120° USB 相机专门做流媒体。

传输方案（**不要用 MJPEG**，每帧数百 KB，4G 会卡成幻灯片）：

```
车身相机 --H.264/RTSP--> mediamtx（车端或云端）--WebRTC--> 手机 <video>
目标码率 1.0~1.5 Mbps @720p15，端到端 150~300 ms
```

云端（`8.163.69.109`）只做信令与必要时的 TURN，媒体优先 P2P 直连以省带宽和延迟。手机端在 `cloud/web/car.html` 里挂一个 `<video>` 即可，不复用 `/ws/control` 控制通道。

### 3.1 相机像素与 4G 推流参数（2026-11-21 用户要求：低延迟、不卡、像素高）

**结论：买 200 万像素（2MP = 1920×1080）就够，4K/5MP 在 4G 上只会换来掉帧。**

> 换算备忘：1MP=100 万像素；720p=1280×720≈92 万（约 1MP）；1080p=1920×1080≈207 万（2MP）；
> 4MP=2560×1440；5MP=2592×1944；8MP=3840×2160（即 4K）。
> 商家标 MP 是**感光元件像素总数**，不等于清晰度 —— 镜头质量、曝光控制和编码码率同样决定观感。
> ⚠️ 购买时警惕「500 万像素插值」：插值是软件放大，真实分辨率仍是 1080p。

| 项 | 建议值 | 理由 |
| --- | --- | --- |
| 传感器分辨率 | **1920×1080（2MP）**<br>（MP = megapixel = 百万像素，2MP = 1920×1080 ≈ 207 万像素） | 手机屏 390~430 pt 宽，720p 已 ≥2 倍冗余；1080p 留裁剪余量 |
| 水平视场 | **120°** | 跟随/直播都要看两侧；鱼眼 150°+ 需去畸变，对人脸/外观有害 |
| 输出格式 | **MJPEG（UVC）或 H.264** | USB2 带宽下 1080p30 无损 YUYV 走不动（≈124 MB/s） |
| 帧率 | 30 fps 能力，实际用 **15~25** | 办公跟随 15 足够；4G 上 25 是上限 |
| 曝光 | **支持手动曝光/固定增益** | 自动曝光会让外观相似度忽高忽低 |
| 低照度 | 索尼 IMX307 / IMX291 一类 | 办公室灯光下噪点少，编码码率也低 |

### 3.2 码率预算（4G 上行才是瓶颈，不是相机）

| 档位 | 分辨率/帧率 | H.264 码率 | 4G 上行要求 | 体验 |
| --- | --- | --- | --- | --- |
| 省流 | 640×360 @15 | 300~500 kbps | ≥1 Mbps | 能看轮廓，看不清脸 |
| **默认** | **1280×720 @15** | **0.8~1.2 Mbps** | **≥2 Mbps** | 流畅，能认人 |
| 高清 | 1920×1080 @25 | 2.5~3.5 Mbps | ≥5 Mbps 且信号好 | 会随信号波动降码率 |
| ❌ 不要 | 1080p MJPEG @15 | 8~15 Mbps | ≥20 Mbps | 4G 必卡 |

**"要高像素"的正确做法**：码流走 720p（省带宽、稳），需要细节时用 **HTTP 抓 1080p/4K 单帧快照**
（一次几十 KB，不占持续带宽）。视频流和抓拍分开，比硬推 4K 流更实用。

### 3.3 延迟预算（端到端 200~350 ms 可达）

| 环节 | 理想 | 说明 |
| --- | --- | --- |
| 采集 | 30 ms | 30 fps |
| 编码 | 10~30 ms | x264 `zerolatency`，关 B 帧 |
| 网络（4G + P2P） | 60~150 ms | 走 TURN 中转再加 50~100 ms |
| 抖动缓冲 | 30~100 ms | WebRTC 自适应，别手动加大 |
| 解码 + 显示 | 20~40 ms | 手机硬解 |

**必须做到的 6 条**：① 用 WebRTC，**不要 HLS**（HLS 天生 3~10 s）；② 关键帧间隔 1~2 s；
③ 关 B 帧、`tune=zerolatency`；④ 云端**只转发不转码**（转码一次 +100~200 ms）；
⑤ 初始 `maxBitrate` 设 1.5 Mbps，让 GCC 拥塞控制自己涨到 3 Mbps；⑥ 手机端 `<video playsinline muted>`。

### 3.4 选购核对清单（2026-11-21，看淘宝页面时用）

页面标题普遍虚标（"500 万 1080P"这类自相矛盾的说法），**以下 5 项必须逐条确认**：

| # | 核对项 | 怎么确认 | 不合格的后果 |
| --- | --- | --- | --- |
| 1 | **支持 MJPG 1920×1080 @≥20 fps** | 问客服要 `v4l2-ctl --list-formats-ext` 截图；或收货后自测 | 只有 YUYV 时 USB2 带宽撑不住，1080p 掉到 ~5 fps |
| 2 | **可见光版（带 IR-CUT/650 nm 滤镜）** | 页面常写"近红外 800~1200 nm（默认发货）"，**必须指定 650 nm 版** | 无 IR-CUT 时白天颜色发紫/偏绿，人脸与外观匹配退化 |
| 3 | **真实水平视场角** | 商家标的"120°/165°"多为镜头级口径；收货后 1 m/2 m/3 m 拉尺实测 | 视场不够 → 跟人时目标出画 |
| 4 | **支持手动曝光/固定增益** | `v4l2-ctl --list-ctrls` 能看到 `exposure_auto` / `gain` | 自动曝光让外观相似度忽高忽低 |
| 5 | **UVC 免驱 + Linux 可用** | 页面写 UVC / 提供 SDK 最好 | 需要装私有驱动，编译麻烦 |

**收货必跑的三条命令**：

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext        # 必须看到 MJPG 1920x1080 20fps 以上
v4l2-ctl -d /dev/video0 --list-ctrls              # 必须看到 exposure_auto / gain
ffmpeg -f v4l2 -input_format mjpeg -video_size 1920x1080 -i /dev/video0 -frames:v 1 ok.jpg
```

**视场实测法**：相机架到实际高度，对准 3 m 外一面墙，在画面左右边缘贴标记 → 量两标记间距 L，
水平视场 ≈ 2·atan((L/2)/3)。例如 L=6.9 m → ≈ 98°。

### 3.5 候选对比（2026-11-21 淘宝实看）

| | A 低价通用 | B 2MP 工业 | **C IMX335 5MP 星光级（推荐）** |
| --- | --- | --- | --- |
| 价格 | ¥52~118 | ¥183~283 | **¥238**（含外壳+支架） |
| 传感器 | 标"500 万"实为 2MP/2K | 200 万 | **IMX335，真 500 万，1/2.8" Starvis** |
| 低照度 | 一般 | 一般 | **BSI 星光级，明显好** |
| 接口 | USB2 UVC | USB(UVC) | **USB2.0 UVC**（5MP 只能低帧/拍照，1080p 视频没问题） |
| 视场选择 | 2.8/3.6mm 未标角度 | 按视场分档 | 2.8mm110° / 2.3mm130° / 3.0mm100° …（无畸变档多） |
| 外壳/支架 | ✅ | ❌ 模块 | ✅ 带支架，1/4 螺丝 |
| 结论 | 先试驱动用 | 可用 | **同价位最优，首选** |

**C 的推荐档位：`500万_2.8mm110° 无畸变 ¥238`**（实际水平 ≈86~96°，足够 0.9~3 m 跟人）；
要更宽选 `2.3mm130°`；**不要选 1.4/1.7/1.8mm 鱼眼**（160~220°，人脸被压小且畸变）。

**USB2.0 带宽现实**：1080p@30 MJPEG ≈ 8~15 MB/s（OK）；2592×1944@15 ≈ 10~20 MB/s（吃紧、可能掉帧）。
→ 视觉用 **1080p MJPEG** 输出，5MP 只用于单帧抓拍；4G 推流本来只推 720p，不受影响。

**下单前问客服三句**：
1. 是否带 IR-CUT 滤光片？白天颜色正常吗？（星光级常配 IR-CUT，但要确认）
2. USB2.0 下 MJPEG 1080p 能到多少帧？2592×1944 呢？
3. 能否用 UVC 手动锁曝光/增益？（`v4l2-ctl --list-ctrls` 是否有 `exposure_auto`）

> **镜头焦距速查**（1/2.9" 2MP 传感器，无畸变矩形镜头）：
> f=2.7~2.8 mm ≈ **90~100° 水平**（适合 0.9~3 m 跟人）；f=3.6 mm ≈ 70~75°（偏窄）；
> 1.4~1.8 mm 鱼眼 180~240° **不推荐**——人脸/人体在画面里被压小且畸变，识别和观感都差。

---

### 3.6 多链路媒体分发：WiFi 先行，4G 后加（改动量≈0）

**原则：视频管线只写一次，链路是"配置"不是"代码"。** 守住下面 4 条，加 4G 时只改配置。

| # | 规则 | 反例（会害你重写） |
| --- | --- | --- |
| 1 | **视频地址由接口下发**，页面不写死 IP | 页面里硬编码 `http://192.168.0.20:8080/video` |
| 2 | 接口返回 `{type, url}`：`type` 支持 `mjpeg` / `whep`，页面按 type 分派 | 只支持一种播放方式 |
| 3 | **只做出站连接**（机器人 → 云端中继），不依赖公网入站 | 指望办公室路由器做端口映射 |
| 4 | 拉流地址带过期时间（TTL），由云端签发 | 地址永久有效、无法切换 |

**链路对照**（协议不变，只换"谁转发"）：

| 场景 | 机器人 → | 手机 ← | 延迟 |
| --- | --- | --- | --- |
| 办公室 WiFi（现在） | 本机 MJPEG/WebRTC 服务 | 直连机器人 LAN IP | 150~300 ms |
| 外网 + 4G（以后） | **出站**推流到 ECS 中继（mediamtx/WHIP） | 从 ECS 拉流 | +50~100 ms |

因为 4G 通常没有公网 IP，**必须走出站中继**；WebRTC/MJPEG 都能这么做，所以现在用哪种都不影响以后。

**4G 接入时的真实改动清单**（预计半天，不含调试）：

| 层 | 改动 |
| --- | --- |
| 硬件 | USB 4G dongle（EC20/EC25 类，RNDIS/ECM）或 M.2 模块 + SIM |
| 系统 | 多一个 `wwan0`；设路由优先级 **WiFi 优先、断线切 4G**；确认已有的 MQTT/WS 长连接不漂 |
| 机器人软件 | 配置里改 `MEDIA_PUSH_URL` / `MEDIA_MODE=relay`，**3 行** |
| 云端 | 起 mediamtx 中继 + nginx 反代（一次性） |
| 手机页面 | **0 行**（地址由接口下发） |

### 3.6.1 ✅ 已实装：WiFi 下手机看 HP60C 画面（2026-11-21）

**机器人上**：`tools/robot_mjpeg_server.py`（aiohttp + cv2，**零额外依赖**，用 `.venv`）
→ systemd 单元 `fwr-media.service`，监听 **8081**（避开 8080：页面里 8080 会被重定向到 `/car/`）。

```ini
# /etc/systemd/system/fwr-media.service
[Unit]
Description=Robot camera MJPEG server (HP60C -> phone)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=manbaout
WorkingDirectory=/home/manbaout/Company_Project/Four-wheel-robot
ExecStart=/bin/bash -lc "source /opt/ros/humble/setup.bash && exec ./.venv/bin/python tools/robot_mjpeg_server.py --port 8081 --fps 12 --quality 70 --fallback-test-pattern"
Restart=always
RestartSec=3
Nice=5

[Install]
WantedBy=multi-user.target
```

| 端点 | 用途 |
| --- | --- |
| `/video` | MJPEG 流（`<img src>` 直接显示） |
| `/snapshot.jpg` | 单帧抓拍 |
| `/healthz` | `{ok, fps, clients, kb, encode_ms, source: camera/test}` |
| `/` | 浏览器自检页 |

- `--fallback-test-pattern`：**相机没起时先出测试图案，真实帧一到自动切走**，不用重启服务
- **实际用法是 `--v4l2 /dev/video0 --v4l2-size 640x642`（见 §3.6.2）**，厂商 SDK 在本机不可用
- 实测（真实 HP60C，2026-11-21）：**7.6~8.9 fps、34 KB/帧 ≈2.1 Mbps**；
  3 秒测试图案 30 帧；抓拍 `/snapshot.jpg` 34.8 KB、往返 **42 ms**（同网段）

**云端**：`GET /api/devices/{id}/media` 返回地址（需登录），配置用 systemd drop-in：

```ini
# /etc/systemd/system/jc-api.service.d/media.conf
[Service]
Environment=JC_MEDIA_HOSTS='{}'   # 留空 = 用状态里车辆上报的 lan_ip（IP 会变，别写死）
Environment=JC_MEDIA_PORT=8081
```

返回 `{"type":"mjpeg","url":"http://<车辆上报的 lan_ip>:8081/video","snapshot":…,"page":…,"ttl":60}`。
以后接 4G 中继**只改这个接口的返回**，页面不用动。

**手机端**（2026-11-21 用户要求，两轮改版后定稿）：手动模式里点**急停右侧的 📷**，
画面**直接嵌在摇杆上方**，**只有画面本身**——没有标题栏、没有 ⟳/✕ 按钮
（再点一次 📷 收起，切换导航模式或页面进后台自动断流）。
打开画面时摇杆自动缩小（`body.cam-on #joy`），不会压住画面。⚠️ **HTTPS 页面会拦 HTTP 视频**（混合内容）→
页面给出提示 +「在新窗口打开画面」；要内嵌就用 `http://` 打开本页。

**时延（2026-11-21 实测，优化前后）**

| 环节 | 优化前 | 现在 |
| --- | --- | --- |
| 相机节点产出 | 7~11 fps | **15 fps** |
| 节点 → MJPEG 服务（DDS） | — | **2.8 ms** |
| 服务端帧龄（`age_ms`） | 100~370 ms | **24~54 ms** |
| 手机端到达节奏 | 7.6 fps，成批/丢帧 | **15 fps，间隔 67 ms，0% 成批** |
| 首字节 | — | **8 ms** |
| 带宽 | 2×（重复帧）≈8.4 Mbps | **4.2 Mbps**（34 KB/帧 @15 fps） |

三个根因与修法（都在本轮改完）：

1. **解析器等深度收齐才出帧**：一帧 = JPEG + 614 KB 深度 + 512 B 补齐，
   深度要 ~100 ms 才到，等于每帧白等 100 ms → 改成 **JPEG 的 `FFD9` 一出现就回调**
   （`iter_frames(..., on_jpeg=)`），深度延后一拍再发。
2. **同一帧被重复回调两次**：JPEG 尾已消费、深度未到时外层又找到同一个 `FFD8` →
   帧率"翻倍"、带宽翻倍 → 改成 `pending` 状态机，**每帧只回调一次**
   （新增两条回归测试钉住）。
3. **服务端 `--fps 10` 限速**：节点 15 fps 与 10 fps 限速拍频，实际只出 **7.5 fps**，
   新帧被丢掉 → 压缩帧本来就是原样转发，**`--fps 0` 不限速**。
4. 附带：原始 `image_raw`（921 KB/帧）和深度（614 KB/帧）**没人订阅就不发、不解码**。

**还能再快的三个档位**（按代价排序，未启用）：

| 方案 | 效果 | 代价 |
| --- | --- | --- |
| JPEG 质量 70 → 55 | 34 KB → ~24 KB，弱 WiFi 下更稳 | 细节略糊 |
| 缩到 480×360（`--width 480`） | ~15 KB，手机解码更轻 | 清晰度下降，手机屏本来就 ~380 px 宽，实际差别小 |
| 换 WebRTC/H.264（mediamtx） | 可到 ~100 ms、码率再降 5~10× | 要装中继、改页面播放器，工作量大 |

**相机 -> ROS 话题**（2026-11-21 新增）：`fwr-camera.service` 跑
`ros2 run fwr_bringup hp60c_uvc_node`，独占 `/dev/video0`，发布

| 话题 | 类型 |
| --- | --- |
| `/robot/camera/color/image_raw` | `sensor_msgs/Image` bgr8 640×480 |
| `/robot/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage`（**相机原 JPEG，零重编码**） |
| `/robot/camera/depth/image_raw` | `sensor_msgs/Image` 16UC1（**毫米**，0=无效） |
| `/robot/camera/{color,depth}/camera_info` | `CameraInfo`（内参未标定，占位 fx=fy=554） |

拆包逻辑与 MJPEG 服务**共用一份**：`fwr_bringup/hp60c_uvc.py`；
回归测试 `tests/test_hp60c_uvc_parse.py`（9 条，钉住整帧消费与深度里假 SOI）。
MJPEG 服务优先订阅 `compressed` 话题（省一次解码、带宽降 27 倍），
`image_raw` 作为 2 s 兜底。

### 3.6.2 HP60C 在这台机器上的真实接入方式（2026-11-21 实测）

**枚举结果**（不是 Orbbec 2bc5，而是 Novatek 桥）：

| 项 | 值 |
| --- | --- |
| USB | `3482:6723 NOVATEK ASJ ZNX_NVT`（high-speed），插在 USB hub 上 |
| 节点 | `/dev/video0`（视频）+ `/dev/video1`（metadata），`/dev/media0` |
| UVC 格式 | `MJPG` 640×642 / 320×564 / 160×768 / 1280×720；`YUYV` 1280×1040@8 |

**一帧的真实结构**（`--stream-to` 落盘逐字节解析得出）：

```
[2560 B 厂商头] + [JPEG 640×480 RGB] + [640×480×2 B 深度] + [512 B 补齐]   ≈ 630 KB/帧
```

- JPEG 用 `cv2.imdecode` 正常解出 **640×480 BGR**（UVC 报的 642 行是含深度用的）
- 深度是 `<u2`（小端 16 位），**单位 0.1 mm**：实测非零像素 18.7%，中位 14041 → **1.40 m**
  （由此得 `mm = raw/10`）；服务 `/healthz` 里的 `depth_mm` 就是这个中位数
- 深度区里会随机出现 `0xFFD8`，**必须先按整帧长度（JPEG+614400+512）消费**，
  否则会被误认成下一帧的 JPEG 头（已修）

**实测吞吐（hub 上，40 帧计时）**：

| UVC 尺寸 | 帧率 | 备注 |
| --- | --- | --- |
| 640×642 | **9.3 fps** | 我们的默认 |
| 320×564 | 13.7 fps | 画面只有 320 宽，不要 |
| 1280×720 | **0.9 fps** | 带宽不够，不可用 |

整帧 630 KB × 9 fps ≈ **45 Mbps**，USB2 hub 上跑不满 25 fps。**插到主机 USB3 直连口能更高**
（待实测；hub 上多设备共享带宽）。

**⚠️ 厂商 SDK 在这台机器上不可用**：`ascamera_node`（`hp60c.launch.py`）能打开相机、
报 `start streaming`，但**恰好 1.003 s 后自己 `stopStreaming`**，之后不出任何帧。
调 `fps:=10` 也一样，`stopStreaming` 在闭源 `libAngstrongCameraSdk.so` 里（无源码）。
**所以本项目改为绕开 SDK，直接读 UVC 裸流**（`tools/robot_mjpeg_server.py --v4l2`）。

**操作红线**：

1. **不要同时跑厂商节点和 UVC 直读**（同一 `/dev/video0`，互相抢）
2. **不要 `kill -9` 厂商节点**：实测会把 `/dev/video0`、`/dev/video1` 一起弄没（USB 仍在，uvcvideo 被解绑）。
   恢复办法：`echo 0 | sudo tee /sys/bus/usb/devices/3-1.3/authorized` → 等 1 s → 再 `echo 1`
3. 相机是要插在 **`3-1.3`** 这个口的，换口后上面的路径要跟着改

**遗留**：`/robot/camera/color/image_raw` 目前没有发布者（厂商节点不可用）。人脸/打卡与跟随
需要 ROS 图像话题时，要照同样的解析写一个「UVC → ROS Image」节点（约 30 行，复用 §3.6.2 的拆包逻辑）。

**4G 特有的三个坑**：

1. **流量**：720p15 ≈ 1 Mbps ≈ **450 MB/小时**。建议"按需推流"——只有手机打开观看页才推，关闭 10 s 后停流。
2. **双网卡路由**：WiFi + 4G 同时在线时默认路由会乱，MQTT 连接可能时断时续 → 用 `ip route` 的 metric 或 NetworkManager 连接优先级，明确主备。
3. **带宽自适应**：用 WebRTC 的 GCC 拥塞控制（初始 1.5 Mbps 自动升降），不要写死码率；信号差时掉到 480p 也比卡住强。


### 3.6.2.1 本机 HP60C 裸流是**左右镜像**的（2026-11-21 确认并修正）

绕开厂商 SDK 直接读 UVC 拿到的 JPEG 是镜像画面（用户用手机相机并排对照确认）。
`hp60c_uvc_node` 参数 **`mirror`（默认 true）** 在发布前把**彩色图与深度图同时左右翻转**，
保证两者仍然对齐、下游（`depthimage_to_laserscan`、跟随链）拿到的是正确朝向。

| 项 | 不翻（默认关） | 翻正（现在） |
| --- | --- | --- |
| 相机节点 CPU | ~9% | **~21%** |
| 单帧 JPEG | 34 KB | **44 KB**（重编码 q92） |

判定方法：把原始帧和当前画面做相关 —— 当前画面与「原始帧左右翻转」相关 **+0.496**，
与原样相关 **−0.221**，即确实翻正了（`/tmp/mirror_probe*.py` 那套自动判定在玻璃多的
场景里不可靠，直接用 v4l2-ctl 抓一帧原始 JPEG 对比最准）。

### 3.6.2.2 手机端再降一档：`--width 480`（2026-11-21 实测）

`fwr-media.service` 加上 `--width 480`，服务端把 640x480 缩到 **480x360** 再发：

| 项 | 640x480 q92 | **480x360 q70** |
| --- | --- | --- |
| 单帧 | 44 KB | **16.8 KB** |
| 带宽（15 fps） | 5.3 Mbps | **2.1 Mbps** |
| 服务端编码耗时 | 0.7 ms（透传） | 6.2 ms（缩放+重编码） |
| 帧间隔 | 67 ms 均值 / 122 ms 最大 | **67 ms 均值 / 82 ms 最大** |

实测 `/healthz`：`fps 15.0`、`age_ms 31`、`encode_ms 6.2`、`kb 16.8`。
想再省就继续 `--width 400`（约 400x300），或把质量降到 60；再往下就该上 WebRTC 了。

### 3.6.3 CPU 与带宽账：**别让相机节点解码原始图**（2026-11-21 实测）

手机推流走 `compressed`（原 JPEG，~34 KB/帧）时，相机节点只做解析转发，CPU **9~11%**。
一旦有人订阅 `/robot/camera/color/image_raw`（921 KB `bgr8`），节点每帧要
`cv2.imdecode` 再经 DDS 发 921 KB —— 15 fps ≈ **13.8 MB/s**，CPU 冲到 **90%**，
主控负载从 3 涨到 **47**，手机摇杆随即"一卡一卡"。

| 状态 | 相机节点 CPU | DDS 图像带宽 | 1 分钟负载 |
| --- | --- | --- | --- |
| 只有 compressed 订阅者 | **9~11%** | **0.5 MB/s** | 3~4 |
| 多了原始图订阅者 | **90%** | **14.3 MB/s** | 47 |

因此 `tools/robot_mjpeg_server.py` 改成**懒订阅**：启动只订 compressed，
**3 秒**仍没收到图像才退回订原始图；压缩话题一到立刻退订。
同理，看门狗查生命周期状态**不要用 `ros2 lifecycle get`**（每次 ~2 s CPU 且高负载下 10 s 超时），
用节点内的 `GetState` 服务客户端异步查（见 `tools/chain_watchdog.py`）。

## 4. 有双雷达了，深度相机还治什么

现有雷达是**一个水平扫描面**（双雷达合并 360°，但仍在同一高度）。深度相机的价值在雷达的物理盲区：

| 场景 | 2D 雷达 | 深度相机 |
| --- | --- | --- |
| 低于扫描面的矮障碍（桌腿、地上的箱子、宠物） | 扫不到，会撞 | ✅ 3D 体素可见 |
| 负障碍（坑洞、台阶下沿、地毯边） | 看不见 | ✅ 地面平面检测 |
| 悬空/横杆（桌沿、管线、门帘） | 看不见 | ✅ |
| 玻璃 / 镜面 / 黑色哑光 | 穿透或吸收 | ⚠️ 也弱，但结构光略好 |
| 窄门贴边通过、精准靠站 | 只有 2D 轮廓 | ✅ 3D 车身包络、毫米级对接 |
| 抓取（桌面分割、物体位姿） | 无 | ✅ 唯一手段 |
| 人/伸手臂避让 | 只有腿部截面 | ✅ 上半身 |
| 远程协助给操作员看 | 无 | ✅ 点云/深度伪彩 |

### 建议的投入顺序（先花小钱）

1. **0 元**：把 HP60C 的 `/robot/camera/depth/image_raw` 接进 Nav2 `spatio_temporal_voxel_layer`（STVL）或 `octomap_server`，先解决"撞桌腿/矮箱子"。
2. **约 ¥150**：120° 广角 RGB + WebRTC，解决远程接管（客户最直观的体验）。
3. **上机械臂时**：夹爪端短距深度（D405 或 Gemini 2L），做抓取。

> CPU 提醒：主控是 i5-13500H，`governor` 当前为 `powersave`，建图时 `slam_toolbox` 已吃满 16 线程。
> STVL 再加两路深度点云约需 1~2 核，**建议先把 governor 改 performance**（此项尚未执行，待批准）。

---

## 5. 推荐配置（单台车）

```
机身前部（现有）      HP60C 深度           → 3D 避障体素层 + 考勤（复用，零成本）
机身中部/顶部         120° 广角 RGB 1080p   → 手机 WebRTC 直播        ¥150
夹爪末端（上臂时）    RealSense D405       → 抓取（eye-in-hand）     ¥3,000
                     预算紧换 Gemini 2L                               ¥1,300
```

原则：**深度不要省成 RGB，但也不要买三台机位** —— 机身那台做粗定位/避障，夹爪那台做精对位，直播用广角 RGB。

---

## 6. ROS 2 集成点（实施时用）

| 目标 | 组件 |
| --- | --- |
| 深度接入 | `realsense2_camera` / `orbbec_camera`（HP60C 已用 `ascamera`） |
| 3D 避障 | Nav2 `spatio_temporal_voxel_layer`（STVL）或 `voxel_layer` + `octomap_server` |
| 抓取 | MoveIt2 + `easy_handeye2`（手眼标定）+ 桌面分割（PCL `SACMODEL_PLANE`） |
| 视频流 | `mediamtx`（RTSP→WebRTC）或 `go2rtc`；手机 `car.html` 加 `<video>` |
| 图像传输 | `image_transport`（`compressed`/`theora`）、必要时 `webrtc_ros` |

---

## 7. 待确认（决定最终选型）

1. **机械臂型号 / 负载 / 工作半径**？末端是两指夹爪还是吸盘？（决定相机体积、安装位、能否用 D405 短距）
2. **抓什么物体**？固定几种（可贴 AprilTag / 用已知模型，方案可很轻）还是任意物体（需 6D 位姿估计，算力翻倍）？
3. **单台 BOM 预算**卡在 ¥1,500 / ¥3,000 / ¥5,000 哪一档？
4. 直播是否需要**双向音频**（远程喊话）？影响是否直接上 WebRTC 而非 MJPEG。

### 3.7 人体识别（跟随模式，2026-11-21）

| 项 | 值 |
| --- | --- |
| 模型 | **YOLO11n ONNX**，`models/yolo11n.onnx`（10.9 MB，COCO 80 类，只要 person） |
| 运行时 | **onnxruntime CPU**（4 线程实测）；cv2.dnn **不可用**（YOLO11 的 Concat 解析失败） |
| 推理耗时 | 320 输入 **13 ms** / 416 输入 **21 ms** / 640 输入 **44 ms** |
| 节点 | `person_detector`（`fwr-vision.service`），默认 **8 Hz**、输入 416、conf 0.35 |
| 常驻 CPU | **~55%** 单核（含 JPEG 解码）。⚠️ 必须 `allow_spinning=0`，否则 319% |
| 话题 | 发 `/robot/vision/persons`，收 `/robot/vision/select` |
| 手机接口 | `GET /boxes`（归一化框 + sel/tracking/stale）、`GET /select?id=N` |

选中目标靠 **IoU + 中心距贪心匹配**维持 id（`vision_boxes.assign_ids`），
中心超出 0.35×框长边就判为新目标 —— 本机 cv2 没有 `cv2.legacy` 的 CSRT/KCF，
真正的视觉跟踪器等做跟随控制时再引入。

实测（注入 4 人的样例图）：4 个框全部检出，置信度 0.87/0.85/0.80/0.58，
归一化坐标与像素位置一致（x 0.06~0.83）。

### 3.8 跟随（2026-11-21 第二版：框大小判远近 + 走廊避障）

```
person_detector ──/robot/vision/persons──▶ person_follower ──/cmd_vel_manual──▶ base_bridge
      ▲                                        │  ▲
      └──/robot/vision/select◀── 手机点框 ──────┘  └──/scan_merged（走廊 ±0.22 m + 侧向 ±30~70°）
```

| 项 | 值 | 说明 |
| --- | --- | --- |
| 停 | 框高 **75%** | 2026-11-21 用户实测：75% 就是合适距离 |
| 起走 | 框高 ≤ **65%** | 迟滞，防阈值上下抖 |
| 后退 | 框高 ≥ **92%** | 退到 85% 停；边退边按框位置转向 |
| 车尾避障 | 走廊 ±0.22 m：0.60 m 减速、**0.30 m 禁退** | 禁退时原地转向对准（车尾超声还会再拦一次）|
| 原地对准 | 目标偏离中线 > **30%**（车头 0.8 m 内有东西时收紧到 **12%**） | 人从车旁走过时车头追着框转，不前冲 |
| 穿门框 | 车前 1.1 m 内左右都有边界、净宽 **0.55~1.05 m** | **改瞄门洞中心、不看框**：偏 >7 cm 先原地站正；门内 0.18 m/s、\|ω\| ≤ 0.45 |
| 整车出门 | 门线纵向 < -(0.23+0.06) = **-0.29 m** | 前雷达在车头 +0.21 m，只看它会"车尾还在门里就转身"；用 360° 合并扫描量门线 |
| 门内丢人 | 门洞还在 + 目标丢失 + 记得最后在画面哪侧 | **先把门走完**（0.18 m/s，≤4 s），出门后**原地朝那一侧转身**找人（0.5 rad/s × 2.5 s ≈72°） |
| 慢走 | 框高 **62%~75%** | 线性插值到 0.25 m/s |
| 全速 | 框高 < 62% | **0.25 m/s** |
| 不追 | 框高 ≤ **30%** | 只转过去看，不前进 |
| 转向 | `yaw_kp 3.5`、上限 **0.9 rad/s**、死区 ±5% | 第一版 2.2/0.6 跟不上人横走 |
| 后退 | **0.12 m/s** | 比前进慢，给避障留时间 |
| 走廊障碍 | 横向 ≤ **0.22 m**：0.35 m 停、0.60 m 降速 | 门框在走廊外 → 能过门 |
| 侧向净空 | 要往哪边转，那边 ≥ **0.30 m** | |
| 丢人 | 0.8 s 停、3.0 s 放弃 | |
| 死手 | 页面 1 s 心跳，2 s 不到就停 | |

手机端画面下方有 **104 px 小摇杆**，碰一下立即退出自动跟随交给人开。
参数全部是跟随节点私有（ROS 参数可覆盖），**不涉及导航的冻结参数**。

### 3.9 第二路相机：120° 广角 5MP（2026-11-25 实测接入）

用户新插了一个广角摄像头，要求手机端在 HP60C 画面**右上角**加切换键。这里记录
硬件识别、USB 分配结论和取流方式。

#### 3.9.1 识别结果（实机）

| 项 | 值 |
| --- | --- |
| 型号 | **Xitech 5MP USB Camera**（USB `0abd:8050`）|
| 节点 | `/dev/video2`（采集）+ `/dev/video3` |
| **稳定路径** | `/dev/v4l/by-id/usb-Xitech_5MP_USB_Camera_20250923107-video-index0` |
| 格式 | **MJPG** 最高 2592×1944@30；**YUYV** 最高 640×480@30 |
| 实测码率 | 1280×720 MJPG = **55 KB/帧**（20 帧 1.06 MB），30 fps ≈ 1.65 MB/s ≈ 13 Mbps |
| 挂载位置 | 总线 3 → 根口 1 → hub1 → hub2（**和 4G 模块同一个二级 hub**）|

⚠️ **别用 YUYV**（2592×1944 下只出几帧且吃满总线）；服务端默认按 MJPG 1280×720 采集。

#### 3.9.2 车端取流：零转码

`tools/robot_mjpeg_server.py` 新增 `WideUvcThread`：

* `v4l2-ctl -d <by-id> --set-fmt-video=width=1280,height=720,pixelformat=MJPG
  --stream-mmap --stream-count=0 --stream-to=-` 拉裸流；
* 裸流只吐负载、不给帧长 → 按 JPEG 标记 `FFD8..FFD9` 切帧
  （`split_jpeg_frames`，实测 70 帧 **0 坏帧**）；
* `hub.publish_jpeg()` **原样透传**，只在 `--wide-width`（默认 480）要求缩小时才解码重编码；
* 默认 `--wide-fps 12`：4G 只有 ~3.5 Mbps，12 fps × 480 宽（约 12 KB/帧）足够看且不挤占。
* 设备按 `*5MP_USB_Camera*-video-index0` 自动找，**不写死 `/dev/video2`**；拔掉会每 3 s 重试。

HTTP：`GET /video?cam=wide` 取广角，`/video`（或不带参数）仍是 HP60C 主摄；
`/healthz` 多一个 `wide` 字段；`/snapshot.jpg?cam=wide` 抓广角单帧。

手机页：手动模式摄像头画面**右上角** `#camSwap`（按钮文字 = 点了会切到哪一路：
主摄显示「广角」、广角显示「主摄」），**只在画面打开时出现**；切换会清 `src` 重连，
断流自愈/3 s 兜底重连都带当前选择（不会悄悄跳回主摄）。

#### 3.9.3 USB 口分配（2026-11-28 已按推荐表重排，实测）

主控 **4×USB-A + 1×USB-C**，8 个 USB 设备。**2026-11-28 重排后**的实际拓扑（`lsusb -t` 实测）：

```
Bus03 Port2 ─ hub(VIA 2822) ─┬─ 3-2.1 前雷达 CP2102N  e00aaf47…  → ttyUSB2
                             ├─ 3-2.2 后雷达 CP2102N  14da82d3…  → ttyUSB5
                             ├─ 3-2.3 电池表 CH340    1a86:7523  → ttyUSB0（udev → /dev/jsy_meter）
                             └─ 3-2.4 IMU DM-IMU-L1  6877:4d55  → ttyACM0
Bus03 Port4 ─ HP60C 深度相机 3482:6723（直连，video0/1）
Bus03 Port7 ─ 5MP 广角相机   0abd:8050（直连，video2/3）
Bus03 Port9 ─ 4G EC200A      2c7c:6005（直连，ttyUSB1/3/4 → ppp0）
```

⚠️ **三条固定路径规则（踩过，别再踩）**
1. **电池表 CH340 的 udev 软链接钉在物理口上**：`/etc/udev/rules.d/99-fwr-serial.rules`
   里 `KERNELS=="3-2.3"`。**换个 USB 口就要改这个值**，否则 `/dev/jsy_meter` 不存在 →
   `battery_modbus` 每秒报 `打开 /dev/jsy_meter 失败` → 手机不显示电量。
   两个 CH340（电池表/超声波）ID 完全相同且**没有序列号**，只能靠物理口区分。
2. **ESP32-S3 必须插板上丝印 `USB` 的那个 Type-C 口**。插 `UART` 口时枚举成
   `10c4:ea60 Silicon Labs CP210x`（和雷达同款芯片），`base_bridge` 只认
   `usb-Espressif_USB_JTAG_serial_debug_unit_90:70:69:F7:F0:B0-if00` → 认不到 →
   `ESP32 serial unavailable` → `conn=False` → 页面「已授权（等待传感器）」。
   插对时 `lsusb` 显示 `303a:1001 Espressif USB JTAG/serial debug unit` → `ttyACM1`。
   `base_bridge` 每 5 s 重试打开串口，**插上即恢复，不用重启服务**。
3. **NetworkManager 的 `4g` 连接不许写死 `connection.interface-name=ttyUSBx`**：
   重插后编号会变（本次 4G 从 ttyUSB5 变 ttyUSB4，而 ttyUSB5 成了后雷达），
   NM 直接报 `No suitable device found … mismatching interface name`，`ppp0` 起不来。
   现在该字段为**空**（`nmcli con mod 4g connection.interface-name ""`），NM 自动找唯一 modem。

**电流账（重排后已解决大头）**：4G 与两台相机都拿到直连口，hub 上只剩低速串口
（雷达×2 + CH340 + IMU，合计 ~450 mA）。

**天线**：2026-11-28 重插后 RSRP **-120 dBm**（正常约 -80）→ 4G 能拨号 `ppp0` 但零回包，
**先查天线**（模块侧 SMA/IPEX 是否接好、天线是否被压在壳内/金属旁）再怀疑套餐。
