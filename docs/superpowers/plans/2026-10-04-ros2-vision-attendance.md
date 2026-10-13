# ROS 2 Vision and Attendance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在独立的二代工程中，让 HP60C 的 RGB/深度流接入 ROS 2，并在开发电脑完成可验证的人脸录入、识别和打卡。

**Architecture:** `ascamera` 作为外部相机驱动；本项目以统一话题和类型化消息隔离相机。视觉节点负责识别，打卡节点负责稳定判定与持久化，本地管理页面负责录入和查询。未来相机、Jetson、底盘和机械臂通过既定 ROS 2 接口接入。

**Tech Stack:** Ubuntu x86_64、ROS 2 Humble、Python 3、rclpy、cv_bridge、项目虚拟环境中的 OpenCV 4.10.0.84 与 NumPy 1.26.4、YuNet/SFace ONNX、SQLite、Flask、Pillow、pytest。

**Spec:** `docs/superpowers/specs/2026-10-04-ros2-vision-attendance-design.md`

## Global Constraints

- 此目录是全新的二代工程；一代仓库只供参考，不复制旧 Flask/MQTT 应用或个人数据。
- 当前先在开发电脑运行；相机为安思彊 HP60C，驱动来自外部 `/home/manbaout/ascam_ros2_ws`。
- RGB 识别在无深度时仍工作；未经实机配准验证，不根据同一像素位置报告距离，也不宣称活体检测。
- 人员照片、特征、数据库、模型及运行日志均留在用户数据目录或受忽略的下载目录，不提交 Git。
- 同一身份在 2 秒内至少 3 次有效识别才打卡；同一人员 60 秒内不重复打卡。参数可配置。
- 本轮只保存设计和计划，实施时也先不初始化 Git、设置远程或提交；待用户以后要求再处理 Git。

## Review Focus

- 相机图像时间戳陈旧或逆序时，不得触发打卡；在 Task 5 测试。
- 数据库录入中途失败时，不得留下半个人员或部分样本；在 Task 3 测试。
- 同帧多张脸、未知人员或接近阈值的身份，不得互相合并或误打卡；在 Task 4/5 测试。
- 深度缺失、编码不支持或未配准时，距离必须为空且 RGB 识别继续；在 Task 4 测试。
- 恶意或损坏的上传文件、重复人员编号和过大图片必须有明确错误，不能损坏数据库；在 Task 6 测试。

---

## File Map

- `ros2_ws/src/fwr_interfaces/msg/{FaceObservation,FaceObservationArray,AttendanceEvent}.msg`：公共数据契约。
- `ros2_ws/src/fwr_vision/fwr_vision/{models,engine,depth,node}.py`：模型加载、识别、深度处理、ROS 2 适配。
- `ros2_ws/src/fwr_data/fwr_data/store.py`：人员和打卡记录的唯一持久化接口，避免视觉与管理页面相互依赖。
- `ros2_ws/src/fwr_attendance/fwr_attendance/{decision,node,web}.py`：打卡判定、ROS 2 适配、本地管理页面。
- `ros2_ws/src/fwr_bringup/launch/vision_attendance.launch.py` 和 `config/*.yaml`：启动与参数。
- `scripts/fetch_models.py`、`scripts/bootstrap.sh`、`README.md`、`.gitignore`：模型获取、开发环境和操作说明。
- `tests/`：纯 Python 单元测试与 ROS 2 接口集成测试；不使用一代仓库的人脸数据。

### Task 1: 独立 ROS 2 工程骨架和消息

**Files:** Create `ros2_ws/src/fwr_interfaces/{package.xml,CMakeLists.txt,msg/*.msg}`、`ros2_ws/src/fwr_data/{package.xml,setup.py,resource/fwr_data}`、`ros2_ws/src/fwr_vision/{package.xml,setup.py,resource/fwr_vision}`、`ros2_ws/src/fwr_attendance/{package.xml,setup.py,resource/fwr_attendance}`、`ros2_ws/src/fwr_bringup/{package.xml,setup.py,resource/fwr_bringup}`、`.gitignore`、`requirements-runtime.txt`、`requirements-dev.txt`、`scripts/bootstrap.sh`。

**Interfaces:** `FaceObservation.msg` 字段为 `person_id`、`display_name`、`recognized`、`score`、`x/y/width/height`、`has_distance`、`distance_m`；`FaceObservationArray.msg` 为 `std_msgs/Header header` 与 `FaceObservation[] faces`；`AttendanceEvent.msg` 为 `std_msgs/Header header`、`person_id`、`display_name`、`score`、`device_id`。未知人员的 `person_id` 为空且 `recognized=false`。

- [ ] Step 1: 写 `tests/test_interface_contract.py`，断言三份 `.msg` 中上述字段和类型存在，并断言运行数据路径被 `.gitignore` 覆盖。
- [ ] Step 2: 运行 `python3 -m pytest tests/test_interface_contract.py -q`，确认骨架尚缺而失败。
- [ ] Step 3: 建立 Python 包和依赖文件；`bootstrap.sh` 使用系统 ROS 2 Humble，创建 `--system-site-packages` 虚拟环境并安装 Flask、pytest、OpenCV 4.10.0.84 与 NumPy 1.26.4；保留系统的 `rclpy`/`cv_bridge`。系统 OpenCV 4.5.4 无法对所选 YuNet 模型完成实际推理。
- [ ] Step 4: 运行接口测试、`source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build --packages-select fwr_interfaces fwr_data fwr_vision fwr_attendance fwr_bringup`；确认测试和构建通过。

### Task 2: 可校验的模型获取与推理适配

**Files:** Create `scripts/fetch_models.py`、`ros2_ws/src/fwr_vision/fwr_vision/models.py`、`tests/test_models.py`。

**Interfaces:** `ensure_models(target_dir: Path) -> tuple[Path, Path]` 下载或复用模型；`load_models(detector_path: Path, recognizer_path: Path) -> tuple[cv2.FaceDetectorYN, cv2.FaceRecognizerSF]` 负责加载。模型固定为 OpenCV Zoo `47534e27c9851bb1128ccc0102f1145e27f23f98` 的 `face_detection_yunet_2023mar.onnx` 和 `face_recognition_sface_2021dec.onnx`，从 `media.githubusercontent.com/media/opencv/opencv_zoo/<commit>/models/...` 下载。SHA-256 分别为 `8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4` 和 `0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79`。

- [ ] Step 1: 写下载器测试，断言缓存命中、哈希错误拒绝、下载失败不留下伪装成完整模型的文件。
- [ ] Step 2: 运行 `python3 -m pytest tests/test_models.py -q`，确认失败。
- [ ] Step 3: 实现临时文件下载、SHA-256 验证、原子替换和 OpenCV 模型加载；缺少模型时输出明确诊断。
- [ ] Step 4: 运行单元测试及一次真实模型下载/加载烟测，检查两个文件大小、哈希和 OpenCV 构造成功。

### Task 3: 本地人员与打卡数据库

**Files:** Create `ros2_ws/src/fwr_data/fwr_data/store.py`、`tests/test_store.py`。

**Interfaces:** `EnrollmentSample(image_jpeg: bytes, embedding: tuple[float, ...])`；`AttendanceStore(path: Path)` 提供 `enroll(person_id: str, display_name: str, samples: Sequence[EnrollmentSample]) -> None`、`templates() -> list[Template]`、`delete_person(person_id: str) -> bool`、`record_event(person_id: str, display_name: str, occurred_at: datetime, score: float, device_id: str) -> int`、`events() -> list[AttendanceRecord]`。`person_id` 匹配 `[A-Za-z0-9_-]{1,64}`，姓名去空白后长度 1–80，样本数 1–5。

- [ ] Step 1: 写 SQLite 临时库测试，覆盖多样本录入、重复编号、删除后模板消失、记录持久化和录入失败事务回滚。
- [ ] Step 2: 运行 `python3 -m pytest tests/test_store.py -q`，确认失败。
- [ ] Step 3: 实现 schema、事务、外键与 WAL；注册照片和特征存入受保护的 SQLite BLOB，删除人员时清除样本，已有打卡记录保留当时人员编号和姓名。
- [ ] Step 4: 运行数据库测试；重开数据库再查记录，确认持久化。

### Task 4: RGB 人脸识别、可选深度和 ROS 2 节点

**Files:** Create `ros2_ws/src/fwr_vision/fwr_vision/{engine,depth,node}.py`、`tests/test_engine.py`、`tests/test_depth.py`、`tests/test_vision_node.py`。

**Interfaces:** `FaceEngine.extract_one(bgr: np.ndarray) -> EnrollmentSample` 要求恰好一张可用人脸；`FaceEngine.analyze(bgr: np.ndarray, templates: Sequence[Template]) -> list[FaceResult]` 输出每张脸的框、身份、分数；`sample_distance(depth: np.ndarray, box: tuple[int,int,int,int], encoding: str, aligned: bool) -> float | None`；节点订阅可配置 RGB/深度话题，在每个处理帧发布 `/robot/vision/faces`，包括空数组，并保留 RGB 采集时间戳。距离处理只在 `depth_aligned=true` 且时间差在配置窗口内启用，默认 `false`。

- [ ] Step 1: 用伪检测器和伪识别器写测试：无人脸、单人、多人、未知、两候选分数接近、坏图像；深度测试覆盖 `16UC1` 毫米、无效零值、未配准和无深度。
- [ ] Step 2: 运行 `python3 -m pytest tests/test_engine.py tests/test_depth.py tests/test_vision_node.py -q`，确认失败。
- [ ] Step 3: 实现 YuNet 检测、SFace 对齐与特征、余弦匹配（默认阈值 `0.363`，最优和次优差值不小于 `0.05`）、按需模板刷新；节点使用 `cv_bridge` 与消息转换，异常帧只记录错误、不输出识别身份。
- [ ] Step 4: 运行三组测试及 `colcon build --packages-select fwr_vision`；用合成 ROS 图像发布一帧，确认话题有带时间戳的消息。

### Task 5: 稳定判定与打卡事件节点

**Files:** Create `ros2_ws/src/fwr_attendance/fwr_attendance/{decision,node}.py`、`tests/test_decision.py`、`tests/test_attendance_node.py`。

**Interfaces:** `AttendanceDecision.observe(frame_time: datetime, observations: Sequence[FaceResult], now: datetime) -> list[ConfirmedFace]`；节点订阅 `/robot/vision/faces`，数据库成功写入后才发布 `/robot/attendance/events`。默认 2 秒窗口内 3 次、60 秒冷却；对同一帧同一身份只计一次；采集时间超过当前时间 2 秒或未来超过 1 秒均拒绝打卡。

- [ ] Step 1: 写判定测试，覆盖 2/3 次识别、60 秒冷却、未知与低分、多脸、同帧重复、乱序及过期帧、相机停帧和数据库写失败。
- [ ] Step 2: 运行 `python3 -m pytest tests/test_decision.py tests/test_attendance_node.py -q`，确认失败。
- [ ] Step 3: 实现按人员隔离的时间窗口、冷却状态和 ROS 2 事件节点；记录 UTC 带时区时间并仅在提交成功后发布事件。
- [ ] Step 4: 运行两组测试、`colcon build --packages-select fwr_attendance`，用合成观察消息验证一次打卡只产生一条数据库记录和一条事件。

### Task 6: 本地录入与打卡管理页面

**Files:** Create `ros2_ws/src/fwr_attendance/fwr_attendance/{web,enrollment}.py`、`ros2_ws/src/fwr_attendance/fwr_attendance/templates/*.html`、`tests/test_web.py`。

**Interfaces:** Flask 只监听 `127.0.0.1`，提供人员录入、列表、删除、打卡列表与 CSV 导出；`extract_samples(images: Sequence[bytes], face_engine: FaceEngine) -> list[EnrollmentSample]` 只接受每张图恰好一张清晰人脸的 JPEG/PNG，最多 5 张、每张不超过 5 MiB。页面不发布运动控制指令。

- [ ] Step 1: 用 Flask test client 和伪特征提取器写测试：成功录入、损坏图片、无脸/多脸、重复编号、超大上传、删除、重启后查询、CSV UTF-8 导出。
- [ ] Step 2: 运行 `python3 -m pytest tests/test_web.py -q`，确认失败。
- [ ] Step 3: 实现简洁本地管理页面、上传限制、表单校验和错误提示；网页展示打卡时转换为本机时区，不将图片与特征写入静态目录或日志。
- [ ] Step 4: 运行页面测试，启动页面并用浏览器检查录入和打卡列表的实际布局与交互。

### Task 7: 一键启动、离线验收与 HP60C 实机验收

**Files:** Create `ros2_ws/src/fwr_bringup/launch/vision_attendance.launch.py`、`ros2_ws/src/fwr_bringup/config/{hp60c,vision,attendance}.yaml`、`tests/test_bringup.py`、`README.md`、`docs/hardware/hp60c-validation.md`。

**Interfaces:** 启动参数 `use_camera_driver` 默认为真；驱动话题映射来自 HP60C 当前实现：`/ascamera_hp60c/camera_publisher/rgb0/image`、`.../rgb0/camera_info`、`.../depth0/image_raw`、`.../depth0/camera_info`。适配层向下游暴露 `/robot/camera/color/image_raw`、`/robot/camera/color/camera_info`、`/robot/camera/depth/image_raw`、`/robot/camera/depth/camera_info`。如实机设备索引不同，通过参数覆盖，不硬编码设备序号或当前用户绝对路径。

- [ ] Step 1: 写启动配置测试，检查参数、话题映射和 `depth_aligned=false` 默认值；用离线图像消息写端到端测试：录入、识别、一次打卡、未知者无打卡。
- [ ] Step 2: 运行 `python3 -m pytest tests/test_bringup.py -q`，确认失败。
- [ ] Step 3: 实现启动配置与 README，写清环境构建、模型获取、离线测试、HP60C 接入、录入及 CSV 导出命令；模型和运行数据路径由环境变量或参数指定。
- [ ] Step 4: 运行全部单元/集成测试和 `colcon build`；在无相机条件下验证离线链路。本机接入 HP60C 后检查 `lsusb`、`ros2 topic list/info/hz/echo`、时间戳、深度格式、RGB/深度配准，再把实际结果写入 `docs/hardware/hp60c-validation.md`。硬件未连接时明确标为未验收，不声称实机通过。

## Plan Self-Review

七个任务分别交付工程接口、模型、数据库、识别、打卡、管理页面和启动验收；每项有独立测试或构建结果。第一阶段的录入、识别、记录、导出、异常帧和无深度行为均有对应任务。导航、机械臂和腰部关节模组只保留未来边界，不在本计划中实现。按用户要求，计划中没有初始化 Git 或提交步骤。
