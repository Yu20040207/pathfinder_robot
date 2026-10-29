# 四轮机器人二代

## 语音交互开发版

语音系统目前在电脑上以 ROS 2 模拟任务运行，支持本地固定指令“停止”“机器人状态”“去大厅”“去实验室”，以及“前进”“后退”“左转”“右转”等模拟底盘动作。开放对话可配置 OpenAI 兼容的文本模型接口；断网时固定指令仍由本地解析。当前没有实机底盘任务执行器，语音节点不会发布实机 `/cmd_vel`。

安装语音依赖并构建：

```bash
./.venv/bin/python -m pip install -r requirements-runtime.txt
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select fwr_interfaces fwr_voice fwr_navigation fwr_bringup --symlink-install
source install/setup.bash
ros2 launch fwr_bringup voice_chassis_demo.launch.py
```

另一个已 source 的终端可发送文本指令并观察回答：

```bash
ros2 topic echo /robot/voice/response
ros2 topic pub --once /robot/voice/text_input std_msgs/msg/String "{data: '去大厅'}"
```

模拟底盘可发送“前进”“后退”“左转”“右转”：收到后持续运动，直到说“停止”。“前进两米”“后退0.5米”会按模拟速度积分的距离自动停止；“前进一下”等指令仍是短动作。动作接受时只回答“收到”，结束时不再重复播报。执行器每 0.1 秒刷新 `/demo/cmd_vel`，取消或到达距离后发零速度；RViz 模拟节点若 0.3 秒没有收到速度也会自动归零。`voice_chassis_demo.launch.py` 在 RViz 中显示底盘、轮子和运动轨迹，并发布 `/demo/odom` 与 `odom → base_link` TF。它是运动学预览，不模拟碰撞、坡道、轮胎打滑或电机负载，也不发布实机 `/cmd_vel`。云端模型不能发起此类动作。另一个终端可用以下命令检查：

```bash
ros2 topic echo /demo/cmd_vel
ros2 topic pub --once /robot/voice/text_input std_msgs/msg/String "{data: '前进一下'}"
```

启动这个集成演示前，先退出旧的 `voice_demo.launch.py`，避免两个语音节点争用麦克风和任务接口。语音唤醒后可说“前进一下”等指令；调试文本话题不需要唤醒。只看 ROS 节点时可加 `show_rviz:=false`，不占用麦克风时可加 `microphone_enabled:=false`。RViz 移动预览使用系统时间，故 `use_sim_time` 默认为 `false`。后续接 Gazebo 或 Isaac Sim 时，仍可沿用本地语音指令与 `/demo/cmd_vel` 控制接口，但应由物理仿真器发布对应的里程计和 TF。

离线 ASR 使用仓库外的 SenseVoice Small ONNX 模型。下载 [sherpa-onnx 官方 INT8 包](https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17.tar.bz2)，解压后将 `voice.yaml` 的 `asr_model_dir` 设为含 `model.int8.onnx` 和 `tokens.txt` 的目录；设置 `input_wav` 可处理一条单声道 16 kHz PCM16 WAV，或设置 `microphone_enabled: true` 从麦克风采集。也可以保持节点的 `microphone_enabled: false`，在另一个已 source 的终端用 `.venv/bin/python scripts/voice_push_to_talk.py --model-dir /模型目录`，按回车录制并提交一次语音。使用麦克风前先确认设备能以 16 kHz 单声道采样。中文 TTS 可下载 [官方 VITS 中文模型](https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/sherpa-onnx-vits-zh-ll.tar.bz2)，将 `tts_model_dir` 指向解压目录；空值时仍发布文本回答。音频设备优先使用 PortAudio，缺少该库时使用系统 `arecord`/`aplay`。`voice_python` 启动参数默认选择项目 `.venv/bin/python`，也可用 `FWR_PYTHON` 覆盖。

云端问答现已预设阿里云百炼 `qwen3.7-flash` 的 OpenAI 兼容接口。启动前在同一终端设置 `FWR_LLM_API_KEY`；不要把密钥写进配置文件或仓库。固定底盘动作、停止、状态和已配置地点任务由本地识别并直接进入模拟任务接口，不等待云端。开放问题才发送转写文本到百炼，关闭思考模式并流式接收回答；当前默认等完整回答后一次提交 TTS 播报。云端回答只作为文本播报，不能发布底盘任务。百炼不可用时仍可使用本地指令。若有百炼工作空间专属域名，可将 `voice.yaml` 的 `cloud_base_url` 换为该域名。模型和录音应放在项目仓库外。

本机已开启麦克风常驻监听，并使用本地识别的“小明同学”作为唤醒词。可说“小明同学，前进一下”，或先说“小明同学”，听到“我在，请说”后连续对话。会话以 30 秒空闲时间计时：每次有效提问或指令、每次回答发出及播报结束都会重新计时；连续对话期间不会按首次唤醒时间退出。空闲超时后会说“我退下啦，有事再叫我”，之后需重新唤醒。在会话中说“退下”“退一下吧”“再见”“拜拜”“结束对话”等也会立即本地结束会话，下一句须重新唤醒。“停止”始终可直接说；已唤醒时只停止底盘动作并重置 30 秒空闲计时，可以接着说下一条指令。无意义的“嗯”“呃”等短词不会提交给大模型，最近播报过的相同文本会被作为回声忽略。调试用 `/robot/voice/text_input` 和按键录音脚本绕过唤醒门，方便检查指令，但不能用它们验证唤醒词。开始常驻监听前请退出按键录音脚本，避免两个进程争用麦克风。唤醒采用 SenseVoice 转写后的文本匹配，首次唤醒仍需等待一次本地语音识别；后续若要求更短唤醒延迟，可换成专用唤醒词检测模型。
`voice_demo.launch.py` 的终端会打印有效麦克风识别文本、文本话题输入和机器人输出。默认只打印最终完整回复；开启 `tts_streaming` 后才逐句打印并播报。修改代码后重启 launch 进程才能看到新的日志。

当前 VITS 模型有 5 个说话人。若需要更多音色，需要换用其他兼容的中文 TTS 模型；仅修改当前模型的 speaker_id 无法增加音色。修改 `ros2_ws/src/fwr_bringup/config/voice.yaml` 中的 `tts_speaker_id` 并重启 `voice_demo.launch.py` 即可换音色：`0=suyingxue`、`1=gunian`、`2=fushiyu`、`3=bingjiao`、`4=bazong`。超出 0–4 会在启动时明确报错；这些名称来自模型自带的说话人配置。
`tts_streaming: false` 默认等待云端完整回答后一次合成播报，减少逐句停顿；改为 `true` 可恢复逐句抢先播报，首句延迟更低。麦克风采集保留起音前约 0.2 秒音频，以减少句首被截断。当前默认 `tts_provider: aliyun`、`tts_cloud_model: cosyvoice-v3-flash`、`tts_cloud_voice: longanya_v3`，使用与百炼问答相同的 `FWR_LLM_API_KEY`。在线时优先播报“龙安雅”音色；未设置密钥或云端语音不可用时自动使用本地 VITS。`tts_cloud_websocket_url` 可替换为百炼工作空间专属 WebSocket 地址。语音合成会按阿里云计费。`tts_speed` 控制语速，当前设为正常语速 `1.0`；本地 VITS 允许范围为 `0.5`–`1.5`，修改后同样需要重启节点。不同 VITS 模型对速度参数的实际响应可能不同。

语音架构见[设计文档](docs/superpowers/specs/2026-10-09-voice-interaction-design.md)，实测方法见[语音评测方法](docs/voice/benchmark-protocol.md)。后续主控目标已定为 Jetson Orin Nano Super Developer Kit（8GB），开发与迁移约束见 [Jetson 主控迁移约束](docs/hardware/jetson-migration.md)；麦克风、扬声器和实机控制接口确定后，还需验证语音与视觉并发、现场识别率及真实急停链路。

这是独立的 ROS 2 Humble 工程。一代仓库仅供硬件协议与业务规则参考。本阶段实现 HP60C 相机输入、人脸录入、识别与本机打卡；底盘路径规划、机械臂按电梯按钮及腰部关节模组在后续阶段接入。

## 开发电脑准备

系统 Python 需要 ROS 2 Humble、`colcon` 和 `cv_bridge`。项目虚拟环境安装 OpenCV 4.10.0.84、NumPy 1.26.4；系统自带的 OpenCV 4.5.4 无法对当前 YuNet 模型实际推理。HP60C 厂商工作区目前位于项目根目录的 `ascam_ros2_ws/`，整个目录被 Git 忽略，SDK 不作为二代源码提交。

厂商工作区是单独获取的依赖。若在本机移动了该目录，先从新位置单独重建相机包（旧 `install/` 内会保留原来的绝对路径）：

```bash
source /opt/ros/humble/setup.bash
cd /home/manbaout/Company_Project/Four-wheel-robot/ascam_ros2_ws
colcon --log-base log_camera build --build-base build_camera --install-base install_camera --packages-select ascamera --symlink-install
```

如需构建厂商工作区的全部 8 个包，运行 `bash scripts/build_ascam_full.sh`。脚本按工作区绝对路径选择独立的 `build_<标识>` 和 `install_<标识>`，避开搬迁前遗留的 CMake 缓存与安装符号链接；构建完成会打印应 `source` 的 `setup.bash` 路径。厂商目录内的 `build.sh` 已在本机作同样调整，但该目录不随二代仓库提交，之后重新获取 SDK 时可使用这里的脚本。

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot
bash scripts/bootstrap.sh
.venv/bin/python scripts/fetch_models.py
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ascam_ros2_ws/install_camera/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

如 Python 包下载源不可用，可设置 `FWR_PIP_INDEX_URL` 后重试。模型保存在 `~/.local/share/four_wheel_robot/models/`，运行数据库默认位于 `~/.local/share/four_wheel_robot/attendance.sqlite3`；这两个目录均不在项目 Git 中。

## 启动

确认 HP60C 已连接，并从项目根目录设置厂商配置目录：

```bash
export FWR_CAMERA_CONFIG_DIR="/home/manbaout/Company_Project/Four-wheel-robot/ascam_ros2_ws/src/ascamera/configurationfiles"
ros2 launch fwr_bringup vision_attendance.launch.py
```

若只检查离线 ROS 2 节点，使用 `use_camera_driver:=false`；不会有实际图像输入。浏览器录入页面需在另一个已 source ROS 2 和本项目 `install/setup.bash` 的终端启动：

若要单独诊断厂商相机，可在一个终端运行 `ros2 launch ascamera hp60c.launch.py`；确认原厂 RGB、深度话题持续出帧后，在另一个终端运行：

```bash
ros2 launch fwr_bringup vision_attendance.launch.py \
  use_camera_driver:=false \
  color_topic:=/ascamera_hp60c/camera_publisher/rgb0/image \
  depth_topic:=/ascamera_hp60c/camera_publisher/depth0/image_raw
```

两个终端都应先按上文 source ROS 2 和工作区。第二条命令不会再打开相机，避免两个驱动进程同时占用 HP60C。当前相机存在间歇性停流，命令可用于隔离诊断，不代表实机识别已通过验收。

`camera_index:=0` 可改为驱动实际设备序号；`db_path:=/绝对路径/attendance.sqlite3` 可覆盖两个 ROS 节点共用的数据库。若覆盖数据库位置，启动网页前也设置同一 `FWR_DB_PATH`。`vision_python` 默认使用项目 `.venv/bin/python`，可用启动参数或 `FWR_PYTHON` 覆盖。

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot
.venv/bin/python -m fwr_attendance.web
```

页面地址为 <http://127.0.0.1:8765>。上传 1–5 张 JPEG/PNG 单人脸照片，填写人员编号与姓名；可查看记录、删除注册样本并导出 CSV。页面只监听本机回环地址。当前版本没有活体检测，不应把打卡结果用作未经人工复核的高风险身份验证。

## 测试与设备验收

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
.venv/bin/python -m pytest tests -q
```

相机的实机出图、RGB/深度同步及空间配准须按 [HP60C 验收记录](docs/hardware/hp60c-validation.md)逐项检查；在确认前 `depth_aligned` 保持关闭。二代设计与后续阶段见 [设计文档](docs/superpowers/specs/2026-10-04-ros2-vision-attendance-design.md)。

2026-10-09 在 USB Bus 1 / Port 2 上有一次成功的约 62 秒采样：RGB 238 帧、深度 75 帧、点云 796 帧，数据非空且非全零。但后续多次启动出现长时间无帧或恢复后再次停流，**稳定性未通过验收**；还需检查画面、深度数值、配准和人脸识别实机效果。详见上述验收记录。

2026-10-08 厂商工作区移至项目根目录后，原有 `install/` 仍引用旧绝对路径。现已单独重建 `ascamera` 到 `ascam_ros2_ws/install_camera/` 并重新构建二代工作区。厂商原启动文件也已修正配置路径及符号链接扫描；修改保存在 [可重用补丁](patches/ascamera-portable-config.patch) 中。

## 路径规划预览

二代已提供独立的地图到路径预览节点。它订阅 ROS 2 标准 `/map`（`nav_msgs/OccupancyGrid`）、`/amcl_pose`（`geometry_msgs/PoseWithCovarianceStamped`）和 `/goal_pose`（`geometry_msgs/PoseStamped`），发布 `/robot/navigation/planned_path`（`nav_msgs/Path`）。地图、起点和终点必须处于同一坐标系；未知区域视作障碍，按 `clearance_m` 膨胀障碍和地图边界。无路可走时发布空路径，避免留下旧路径。

```bash
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
ros2 launch fwr_bringup navigation_preview.launch.py
```

此节点**不发布 `/cmd_vel`，不会驱动底盘**。本机尚未安装 Nav2；驱动轮计划直径虽已给出，但实测滚动半径、轮距、编码器/IMU 与急停接口尚未接入。路径预览不能替代可行驶的导航系统。后续按 [第二阶段首步计划](docs/superpowers/plans/2026-10-05-navigation-preview.md)接入底盘和 Nav2。当前 `clearance_m: 0.25` 仅供离线预览，须在实测机器人外廓后重新标定。

平整地面、低功耗优先时的底盘方案见 [底盘驱动方案](docs/hardware/chassis-drive-selection.md)。当前采用左右两路差速驱动，配前端左右两只无动力万向脚轮；底盘外廓为长 52 cm、宽 35 cm、总高 20 cm。样机先用两台 24 V 立盛 JGY4632-370-2485 的 77.7:1 档编码器减速电机，配直径 100 mm、宽 35 mm 驱动轮；满载 24 kg 时的速度、爬坡能力、连续运行温升和轴承载荷待实测验收。接下来优先打通 HP60C 持续出帧和双机械臂 CAN 总线舵机接口。

后续建图与自主导航的传感器架构见 [底盘激光雷达与导航架构](docs/hardware/chassis-lidar-navigation-design.md)：样机先在底盘前缘固定一只 2D 雷达，配合轮编码器、底盘 IMU、建图/定位和 Nav2。前置单雷达不保证周围 360° 无盲区感知；后置雷达是否需要待实测决定。在后方感知与安全验证完成前，不启用自主倒车；目前仅记录设计，不表示已接入雷达或实机导航。

腰部折叠六舵机、USB-CAN 调试卡和 ESP32-S3 收发器的候选方案及协议疑点见 [腰部 CAN 执行器选型记录](docs/hardware/waist-can-actuator-selection.md)。此方案尚未完成型号核对、机构扭矩计算和实机验证。

当前 Ubuntu 电脑充当开发阶段的 ROS 2 主控；后续迁移至 Jetson Orin Nano Super Developer Kit（8GB）。新功能应保持主机与 ESP32-S3、雷达、IMU 的接口可移植，避免写死电脑设备路径，并在 Jetson 上重新验证 ARM64 依赖、厂商驱动及并发性能，详见 [迁移约束](docs/hardware/jetson-migration.md)。

### RViz 底盘示意模型

当前没有整机 CAD。`docs/hardware/preview/fwr_chassis_preview.urdf` 按用户提供的底盘外廓长 52 cm、宽 35 cm、总高 20 cm 绘制为圆角长方体；两只主动轮位于内部并靠左右边，两只可自由转向的从动轮并排位于前端。主动轮按计划直径 100 mm、宽 35 mm 显示，前脚轮尺寸和全部轮子的精确安装位置仍是可视化占位值，不能直接用作导航 footprint、碰撞检测或制造依据。运行以下命令可在独立 ROS 通信域中打开 RViz：

预览和 `fwr_bringup` 演示 URDF 还包含底盘顶部靠前的 RPLIDAR C1 示意件及固定坐标系 `laser_front`（相对 `base_link`：前 0.19 m、扫描面高 0.2298 m）。示意件外廓按 C1 机械图约 55.6×55.6×41.3 mm 绘制，细节形状及底盘安装位仍是近似，须在实物上核对；当前没有雷达驱动或实机扫描数据。

```bash
./scripts/show_chassis_preview_rviz.sh
```

窗口关闭时，示意模型发布进程也会退出。实机几何参数测量后再替换这些占位值。
预览脚本默认使用软件渲染；需要使用本机图形驱动时可设置 `LIBGL_ALWAYS_SOFTWARE=0`。圆角视觉网格由 `scripts/generate_chassis_preview_mesh.py` 生成，碰撞外形仍是长方体近似；每次启动都会更新本机网格路径，方便后续迁移。外壳在 RViz 中略微透明，便于看到位于内部的轮子。前从动轮有绕竖直轴转向和绕轮轴滚动的关节，预览脚本会自动播放示意动作；这是外观动画，不是万向轮的动力学仿真。

### 差速运动离线演示

下面的演示自动生成障碍地图、起点和目标，规划路径并让虚拟差速机器人绕行到目标。所有话题均在 `/demo` 下，不读取串口，也不发布 `/cmd_vel`：

```bash
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
ros2 launch fwr_bringup navigation_demo.launch.py
```

另开终端可用 `ros2 topic echo --once /demo/reached` 看是否到点，或查看 `/demo/path` 和 `/demo/pose`。2026-10-08 本机运行时得到 16 点路径，虚拟机器人约 29 秒后到达目标附近并停下。差速积分与演示控制器仅供软件验收，完整设计见 [离线演示计划](docs/superpowers/plans/2026-10-05-differential-demo.md)；实机仍需底盘反馈、急停及 Nav2 控制器。

## AI 协作日志

每次 AI 会话改了什么、实测结论、留下了哪些进程和未完成事项，都追加在 [docs/AGENT_LOG.md](docs/AGENT_LOG.md)。接手任务前先读最后几条，再回到本文档和相关设计文档；仓库外的环境改动（例如 DSH 的 `~/.local/bin/systemd-run` 包装脚本）也记在那里，换机器时需要逐项复原。
