# AGENTS.md — 四轮机器人二代

本文件是给 AI 编码代理和新同事的“开工说明”，DSH / Codex 等会在会话开始时自动加载。
**这里只放长期有效的规则、入口和当前快照；一次性的排查过程写到 `docs/AGENT_LOG.md`，计划进度写到 `.superpowers/sdd/<plan>/progress.md`。**

## 1. 项目速览

- ROS 2 Humble + Python 3.10（项目虚拟环境 `.venv/`）+ ESP-IDF。
  **实机主控 = Intel i5-13500H 迷你主机（x86_64 / 16 线程 / 15 GB / Ubuntu 22.04.5），不是 Jetson**
  （2026-11-20 实测；此前文档里的 Jetson Orin Nano Super 计划已作废，仅作历史参考）。
- 主工作区 `ros2_ws/`：`fwr_bringup`、`fwr_voice`、`fwr_navigation`、`fwr_imu`、`fwr_interfaces`、`fwr_vision`、`fwr_attendance`。
- 实机：ESP32-S3 底盘控制、DM-IMU-L1、RPLIDAR C1、HP60C 相机；固件在 `firmware/`。
- 当前分支 `codex/real-nav2-imu-turn`，工作区可能有很多未提交/未跟踪改动。**不要用 `git checkout`、`git reset`、`git clean` 清掉别人的工作。**
- `README.md` 部分段落早于实机进展（例如“尚未安装 Nav2”）；**以 `docs/AGENT_LOG.md` 最新条目为准**。

## 2. 开工顺序（每次任务都做）

1. `git status --short` + `git log --oneline -5`：确认现状，不要假设“上次已经提交”。
2. 读 `docs/AGENT_LOG.md` **最后 2–3 条**（`tail -100 docs/AGENT_LOG.md`），不要整篇读；重点是“已验证/未验证/当前运行状态”。
3. 若任务属于某个计划：读 `.superpowers/sdd/<日期-计划名>/progress.md` 和对应 `docs/superpowers/plans/*.md`。标为 `complete` 的任务不要重做，除非出现新证据。
4. 先 `grep` / `glob` 定位，再按行范围 `read`；不要通读 `build/`、`install/`、`log/`、`.venv/`、`ascam_ros2_ws/third_party/`、`data/`、`models/`。
5. 涉及实机时先查 `ls /dev/serial/by-id/`、`ps -ef | grep -E 'ros2|sllidar|dm_imu|base_bridge'`；不要重复启动同一节点。

## 3. 当前已验证快照（2026-11-21）

> **主控硬件（2026-11-20 实测，取代旧的 Jetson 说法）**
> - CPU：Intel Core i5-13500H（13 代，12 核 16 线程，最高 4.7 GHz），内存 15 GB，Ubuntu 22.04.5 x86_64
> - **没有** `nvpmodel` / `jetson_clocks` / `/etc/nv_tegra_release` —— 不是 Jetson 平台
> - CPU governor = **powersave**（省电降频）；`/sys/.../cpu0/cpufreq/scaling_governor`
> - 后果：建图时 `slam_toolbox` 报 `options.num_threads: 50 exceeds maximum 16`，Ceres 占满核，
>   前雷达 `sllidar_node` 出现过 2.6~3.0 s 静默（进程活着、USB 无断开），由 `fwr-lidar-watchdog` 杀掉重拉。

- **雷达：前后双雷达（2026-11-18 起，实机在跑）**
  两只都是 RPLIDAR C1、460800、`scan_mode Standard`、10 Hz，全部用 **by-id** 路径
  （`/dev/ttyUSB*` 会因插拔/复位重新枚举，**不要写死编号**）：

  | 位置 | 节点 | frame | 话题 | by-id 尾段 |
  | --- | --- | --- | --- | --- |
  | 车头 | `sllidar_node` | `laser_front` | `/scan` | `e00aaf475431...` |
  | 车尾 | `sllidar_node_rear` | `laser_rear` | `/scan_rear` | `14da82d39131...` |

  - 安装位（`description/fwr_chassis_demo.urdf`）：前 `xyz=(0.21, 0, 0.0425)` rpy yaw **π（倒装 180°）**；
    后 `xyz=(-0.229, 0, 0.0425)` rpy yaw **0**
  - 两路各经 `scan_mapping_filter` 遮罩 **±80°**（车身自击段）→ 每路有效 **200°**；
    前保 base_link ±100°、后保 80~280°，**并集 360°**
  - `scan_merger` 合成 **`/scan_merged`**（base_link 系，跨度 359°）：
    slam_toolbox、AMCL、手机扫描图都吃这一路
  - 单雷达回退 `start_rear_lidar:=false` 时才退回 `/scan_mapping`（老的 260° 视场）
  - **改视场必须改 launch**（`real_mapping.launch.py` 的 `rear_half_angle_rad`），
    改节点默认值无效（launch 传参覆盖）。2026-10-31 就因为 launch 写死 π/2，长期停在 180°
  - slam_toolbox **等扫描就绪再启动**（`tools/wait_ready.py` 盯扫描频率，达标后
    `OnProcessExit` 才起；2026-11-24 之前是写死的 `TimerAction 35 s`）。
    原因仍是等两路+合并稳定，否则首帧只有半圈、车不动时地图永远缺一半；
    实测 5~8 s 就稳，白等 27 s
  - `fwr-lidar-watchdog` 管**两路**（`--stale 2.5 --cooldown 15`），任一路断流就杀对应节点等 launch 重拉；
    remaps 必须用配对写法 `(('/scan','/scan_rear'),)`，扁平写法会让看门狗自己崩（2026-11-20，一天 67 次）

  单雷达回退时仍生效的老约束：
  - `/scan` 的后方扇区（scan 角 0° 附近 = ranges 中部）读到的是**车身**，不是障碍；
    `mask_rear_ranges` 屏蔽这一段
  - 任何拿 `/scan` 判断"是否被堵 / 能不能转向"的代码**必须忽略后方扇区**，
    否则会把自己车体当成墙（自动建图曾因此误判"三面被堵"死循环）
  - 倒装：`/scan` 角度范围 [-180°,+180°]，**index 0 侧 = 车头**，index 中部 = 车尾
- **设备路径**（2026-11-03 实测更正，原文档写反了）：
  - ESP32-S3 底盘 = **`/dev/ttyACM0`**（`ID_VENDOR=Espressif`），115200，
    `BASE_CONTROL_V2`，`S2` 为 14 字段
  - IMU DM-IMU-L1 = **`/dev/ttyACM1`**（`ID_VENDOR=DM-Tech`），921600
  - ⚠️ **ACM 编号会随插拔/重启变化**，`base_bridge` 用的是
    `DEFAULT_DEVICE`（by-id：`usb-Espressif_USB_JTAG_serial_debug_unit_...`），
    **不要写死 ACM 编号**。烧录时要用 `idf.py -p <正确的 ACM>`，
    端口被 `base_bridge` 占用时会报 `Invalid head of packet`。
- Nav2 / AMCL / EKF / Collision Monitor 已能运行；`real_navigation.launch.py use_saved_map:=true` 曾连续运行 23 小时以上。
- 已知坑：旧的 `sllidar_node` 可能残留成僵尸发布者，导致 `/scan` 有 2 个 publisher；重启导航栈前先清理。
- `base_bridge` 默认 `motion_enabled=false`、`manual_enabled=false`，不会让车动。

### 🧩 开机 = 手动模式；模式栈按需启动（2026-11-24 用户要求，已实机验证）

用户原话：「开机之后就先切到手动模式…要切换模式的时候就启动其他模式的，
**切模式时先把现有的模式清理干净**」。为此把"硬件"和"算法"分开：

| 服务 | 开机 | 内容 |
| --- | --- | --- |
| **`fwr-base`** | **enabled（常驻）** | `real_base.launch.py`：base_bridge、双雷达、IMU、EKF、scan_merger、rear_guard、静态 TF |
| `fwr-nav` | **disabled（按需）** | `real_navigation.launch.py start_base:=false`：AMCL/map_server/Nav2/collision_monitor |
| `fwr-map` | disabled（按需） | `real_mapping.launch.py start_base:=false start_slam:=true` |
| `fwr-vision` / `fwr-follow` | disabled（按需） | 人体识别 / 跟随控制 |
| `fwr-lidar-watchdog` | enabled（带 `--no-spawn`） | 只负责"发现活着但不出数据的雷达 → SIGKILL 让 fwr-base 的 respawn 重拉"；**不允许自己 `ros2 run sllidar_node`**（会和 fwr-base 抢串口、造出两个 publisher） |

- 硬件节点在 `real_mapping.launch.py` 里用 `start_base` 门控（`_both()` 组合其它开关），
  **只有一处节点定义**，`real_base.launch.py` 只是 `start_slam:=false` 的薄包装。
- 导航栈的 `scripts/run_navigation.sh` 在 `FWR_START_BASE=false`（`fwr-nav.service` 里设）
  时**跳过整套"清理旧栈"**：否则它会 pkill 掉 fwr-base 的 `base_bridge`，
  再因为"进程又回来了"误判旧栈仍在 → `exit 1` → 重启循环（2026-11-24 10:48 踩过）。
- ⚠️ **云端 `cloud/api/app.py` 的 `/ws/control` 是「字段白名单」**：`nav` 类消息只转发
  `action` / `name` / `map`（2026-11-24 补了 `mode`）。**新增任何 nav 字段都必须同步加到这里**，
  否则手机发了、车端收不到——现场表现是「点导航模式，等 1 min+ 一直等待导航栈就绪」，
  车端日志却只有 `已切到手动模式`。诊断方法：在车端订阅 `robot/<id>/nav` 看报文。
- ⚠️ **车端未知模式一律忽略、不许回落成手动**：`_switch_mode` 收到空/非法 mode 只打
  `忽略未知模式`，保持现状（旧写法回落 manual，等于把"切导航"执行成"停车"）。
- 切模式由 `mqtt_bridge` 的 **`mode` 动作**（手机发 `{type:'nav',action:'mode',mode:...}`）：
  1. 确认 `fwr-base` active（不在就拉起来）；
  2. **停掉所有不属于目标模式的栈**，每个都等 systemd 真报 `inactive/failed`；
  3. 再启动目标栈；4. 状态上报 `mode` = `manual|nav|map|follow`（`_sync_mapping_mode` 每 5 s 按真实服务状态对账）。
- ⚠️ 新增 launch 文件后必须 `colcon build --packages-select fwr_bringup --symlink-install`，
  否则 `ros2 launch` 报 `file 'xxx.launch.py' was not found in the share directory`。
- ⚠️ systemd 单元必须带 `Environment=HOME=/home/manbaout`（+ `ROS_LOG_DIR`/`FASTRTPS_...`），
  缺 HOME 会让节点报 `failed to configure logging: Failed to get logging directory` 后 abort
  （2026-11-24 fwr-base 起来就崩，就是这个）。
- 实机验证（2026-11-24）：`mode=map` → nav 停/map 起、**base_bridge 始终 1 个**、
  `/scan_merged` 10 Hz；`mode=nav` → map 停、amcl active、安全链输出 20 Hz；
  `mode=manual` → 导航/建图全停、只剩硬件层、扫描 10 Hz。
- 🗺 **"保存地图"不许抢占当前导航地图；重新定位只认 `map_server` 加载的那张**（2026-11-24 现场定版）
  - `mqtt_bridge._on_map_edit`（手机地图编辑器/建图保存走的都是它）：
    **建图模式（`_mapping_mode` 或 `mode=map`）只写 `maps/phone/<name>.{pgm,yaml}`，
    不改 `~/.config/fwr/last_navigation_map`**；导航模式里的编辑仍更新指针（那是显式改当前图）。
  - 症状回顾：保存动作改写了状态文件 → 切回导航时 `map_server` 起的是**新图**，
    而手机图面还是上一张（地图图片只在 `set_map` 时上传）→ 看起来像"重新定位匹配错了地图"。
  - 因此 `_switch_mode(target='nav')` 结束时会 `_init_active_map()` +
    `_publish_map_file(_active_map_path)`，保证**手机图面 = `/map`**；`set_map` 那条路
    原本就有 `_publish_map()` 兜着，别再重复加。
  - `auto_initial_pose` 的地图来源就是 `/map`（没有第二条路），所以**要换导航图必须换掉
    `last_navigation_map` 并重启 `fwr-nav`**；换图时日志会打
    `地图 WxH @res m（换图，掩码已清）`，重新定位成功日志带同样的地图指纹。
  - ⚠️ `auto_initial_pose._on_map` **无条件清 `_mask_cache`**：换**同尺寸**的另一张图
    时如果不先清空，会拿旧地图的墙面掩码去匹配新图（旧写法只在尺寸变化时清）。
  - 回归：`tests/test_map_save_scope.py`（4 例）。现场核对用
    `grep -o "yaml_filename[^,]*" /tmp/launch_params_*` 看 `map_server` 到底加载了谁。
- 📷 **导航模式也有摄像头（2026-11-24 用户要求）**：`#navCamBtn`（右侧竖排按键第一颗，
  排在「重新定位 📍」上面；整列 `#zoomCtl` `bottom` 86→**62 px**）→
  `#navCamPane`（盖在地图上的悬浮层，`z-index 40`，地图编辑器 60 在它上面）。
  复用同一路 MJPEG `/video`，但**独立一个 `<img>`**（手动模式那块在布局流里，不能共用）。
  切出导航模式会在 `applyMode()` 里自动关掉；断流自愈 / 切后台断流与手动模式同款。
  画面本体挂在 `#navTopWrap` 里（绿色状态行下面），**不用写死的 top**。
- 🗺 **顶部两行是透明浮层，直接压在地图 / 摄像头画面上**（2026-11-24 用户最终定版）：
  `#navView` 仍是**竖排 flex**，但 `#navTopWrap` 是
  **`position:absolute; left/right:0; top:0; background:transparent; pointer-events:none`**
  ——**没有框、没有底色**，只浮两行字（白字行 + 绿字行）。
  `#mapCanvas` `order:2; flex:1 1 auto; min-height:0` 因此**铺满整个 #navView、从最顶上开始画**；
  打开摄像头时 `#navCamPane`（`order:0`）先占位，画布自动排在它下面吃掉剩余高度。
  ⚠️ **文字压在白色地图上必须靠描边**：两条子行都有
  `-webkit-text-stroke:1.1px rgba(0,0,0,.78)` + `paint-order:stroke fill` + 一层浅 text-shadow
  （2.2px 太粗，用户 2026-11-24 让改细；再调就动这一个数字）。
  Safari/WebKit 的 `paint-order` 让描边画在字心下面，字不会变细；改动这两条文字样式时别把描边删了，
  否则白底上白字直接看不见。
  ⚠️ **`pointer-events:none` 不能删**：否则浮层会吃掉地图的捏合/拖动/点击。
  ⚠️ 历史：11-24 早些时候用户要求"地图画布从提示条下面开始画"、提示条带一块扁平深色底；
  **之后又被用户改成"像摄像头画面那样完全透明"**。以本条为准，不要照旧记录改回去。
  也不要再改回"绝对定位浮层 + JS 量高度补偿"（`--navTopH` / `layoutNavTop` 已废弃）：
  量高度永远有窗口期（隐藏时量到 0、画面打开时挤没地图）。
  两条子行都是 **`white-space:nowrap` + `text-overflow:ellipsis`，强制各占一行**（用户明确要求，
  不许换行——换行会把整块顶高、地图跟着跳）。绿字行**不显示位置坐标**（`updateNavStatus()`
  只拼 `状态 · 地图 · 授权`，坐标看地图车标即可）。
  摄像头画面 `#navCamPane` 同样**直角、无边框无阴影**、宽度顶到两边。
  ⚠️ **画面上没有 ✕ 关闭按钮了**（2026-11-24 用户要求删掉）：开关都靠右侧那颗 📷
  `#navCamBtn`——点一下开、再点一下关（`onclick` 里按 `navCamWanted` 分流）。
  📷 **打开画面时画面移到最上面、两行提示浮在画面上**（用户要求）：`#navCamPane{order:0}`
  排在提示条(1)/画布(2) 前面，同时给 `#navView` 加 `.cam-open`，
  ⚠️ **`#navCamPane` 必须是 `#navView` 的直接子元素**（原来的写法塞在 `#navTopWrap` 里，
  那样 `order` 只在提示条内部生效，画面永远排在文字**下面**——2026-11-24 现场截图就是
  「文字一条带、视频在它下面」）。
  让 `#navTopWrap` 变 `position:absolute; top:0` 盖在画面上。关掉画面要摘掉 `.cam-open`。
  ⚠️ 这个状态下提示条**背景必须完全透明**（用户明确要求：只显示文字，背景就是视频），
  靠 `text-shadow`（4 段深色阴影）模拟描边保证在明亮/杂乱画面上还看得清；
  **不要**给它加半透明底色。
  提示条是 `pointer-events:none`，所以浮在上面也不会挡住画面的 ✕ 关闭按钮。
  画布尺寸变化由 `ensureNavCanvas()`（尺寸真变了才 `resizeCanvas()`）在
  切模式 / resize / 每 1.5 s 兜底一次。
- 🗑 **左下角「1 m」比例尺图例已删**（2026-11-24 用户要求）；要恢复就在
  `render()` 里重新加回绘制段（原实现：`nice` 取 0.5/1/2/5/10/20 里 ≤130 px 的最大值）。
- 实机 RViz 的“两点导航”面板：**单击一次**表示从当前位置前往点击点（`/explore/start_fixed_goal`）；**连续点击两次**表示先到起点再到终点（`/explore/start_two_point_goal`）。现场人员点击“开始导航”时面板自行开启 `motion_enabled`、等待 `/base/ready`、申请 `/base/arm_auto`，然后发送目标；点击“停止导航”或任务结束/失败时请求停机并关闭运动门。无需另开终端运行授权脚本。该按钮流程已构建和通过单元测试，**尚未由现场人员验证完整实车按钮操作**；最新证据见 `docs/AGENT_LOG.md` 末尾。代理不得代替用户点击按钮或打开运动门，除非本次会话另有明确授权。


### 当前已验证参数快照（2026-10-30，换橡胶轮后）

**底盘/固件**（`firmware/esp32s3_base_control/main/control.c`）
- `MAX_DUTY 0.90`、`BASE_DUTY 0.60`、`DUTY_RAMP_PER_S 0.5`
- 失速：`STALL_SPEED_MMPS 4`、门槛 12% 指令、`STALL_LIMIT_S 2.0`
- 超速：`OVERSPEED_MMPS 700`、窗口 0.4 s

**里程计**（`base_bridge_core.py`）
- ⚠️ **`track_m 0.317`**（2026-11-11 换 WSDC2412D-V3 + 100 mm 轮 + 卷尺实测 317 mm；
  旧值 0.262 / 0.268 作废）、`circumference_m 0.314159`、`counts_per_rev 6400`
- 任何"单轮包络"计算（`follow_core.limit_pair`、`base_bridge.manual_velocity`）
  **必须用 0.317**，用旧值会把上限算小 1.3 cm/s → 底盘整帧丢弃 → "跟随中但车不动"
  （2026-11-21 实机 11 次 `Rejected out-of-range manual velocity`）。有单测钉死。
- 3 米实测误差 +0.20%（换胎后标定）

**IMU 航向修正**（`fwr_imu/node.py` + launch `imu_yaw_scale`）
- `yaw_scale 1.019`（雷达互相关实测：转一圈真值 362.58°，陀螺积分 355.84°）
- **EKF 吃的是 `/imu/data_raw` 的 vyaw**，姿态四元数不进 EKF；
  换 IMU/改安装后必须重标此系数

**AMCL**（`real_localization.yaml`）
- `max_beams 120`、`laser_max_range 8.0`、`min/max_particles 1000/5000`
- `recovery_alpha_slow/fast = 0/0`（**关闭随机粒子注入**，否则车不动位姿乱跳）
- `update_min_d/a 0.10`

**Nav2**（`real_nav2.yaml`）
- 巡航 `desired_linear_vel 0.15`、前视 `lookahead_dist 0.55`（0.35~0.75）
- 旋转 shim `angular_dist_threshold 1.2 rad`（<0.6 会“转过头”）
- `use_collision_detection: false`（RPP 预测误报太多，由 collision_monitor 兜底）
- 膨胀 `inflation_radius 0.30`、`cost_scaling_factor 2.0`
- Smac `cost_travel_multiplier 8.0`（本版本**没有** `cost_penalty` 参数）
- `front_stop` 半宽 0.24 m、3 点触发、`simulate_ahead_time 0.5`

**已知 Nav2 坑（务必知道）**
- `collision_monitor` 触发一次 `stop` 后会**卡死不再处理指令** →
  `/cmd_vel_auto_safe` 恒为 0、车不动。用 `tools/chain_watchdog.py` 自动重启，
  或手工 `ros2 lifecycle set /collision_monitor_forward deactivate/activate`。
- ⚠️ **监控"一直是 inactive"也要管（2026-11-22）**：雷达在启动期反复超时重启时，
  `collision_monitor` 首次 activate 会因 TF 未就绪失败 → `lifecycle_manager` 放弃 →
  节点**永远停在 inactive**，`/cmd_vel_auto_forward_filtered` 恒无 → 手机端
  `chain=false`，一直显示「等待安全链（碰撞监控）就绪…」（实测 40 分钟没人恢复）。
  旧看门狗只判"输入 20 Hz + 输出静默"，且被"不是 active 时没输出正常"早退放过。
  现在 `chain_watchdog._watch_inactive()`：非 active 持续 > `inactive_grace_s 90 s`
  就补 `configure/activate`（30 s 冷却、最多 3 次、**只补激活不动车**）；
  launch 里看门狗加了 `respawn=True` + `PYTHONUNBUFFERED=1`（它的日志以前被
  4 KB 缓冲吞掉，journald 一行都看不到）。改完**要重启 `fwr-nav` 才生效**。
- 底盘 `motion_enabled=false` 时所有速度指令被丢弃；
  面板“开始导航”会自己开门，但**手动测试时必须先设 true**。

### 工具环境（2026-10-24）

- 用户级 skill：`~/.agents/skills/ros2-engineering-skills`（Apache-2.0，DSH 已识别）。
- ROS MCP：`~/.local/share/fwr-ros-mcp/venv`（`ros-mcp 3.1.2`）+ 只读包装 `~/.local/bin/ros-mcp-readonly`；DSH 的 `mcp-ros` 行已启用，HMR 热加载 11 个只读工具。
- `rosbridge 2.0.8` 监听 9090；连接地址 `127.0.0.1:9090`，端到端只读验证通过（35 节点、`/scan` 720 点）。
- 机器人上 `fwr-media.service`（2026-11-21 起）监听 **8081**，把相机画面转 MJPEG 给手机
  服务端跑 `tools/robot_mjpeg_server.py --fps 0 --quality 70 --width 480 --fallback-test-pattern`
  （`--width 480` = 送到手机的是 **480x360 / 16.8 KB / 2.1 Mbps**；去掉它则按原图
  640x480 / 44 KB / 5.3 Mbps 送。`--width` 是**唯一**需要服务端二次编码的路径，实测 encode 6 ms）：
  **`--fps 0` 千万别改成 10**——节点 15 fps 与限速拍频后实际只出 7.5 fps，延迟翻倍。
  `--fallback-test-pattern` 会在相机没起时先出测试图案。延迟数据见
  `docs/hardware/vision.md`（DDS 2.8 ms、服务端帧龄 24~54 ms、手机 15 fps/67 ms）。
  相机由 **`fwr-camera.service`**（`ros2 run fwr_bringup hp60c_uvc_node`）独占 `/dev/video0`，
  发布 `/robot/camera/{color,depth}/image_raw` + `color/image_raw/compressed`（原 JPEG）；
  MJPEG 服务只订阅 ROS 话题，优先走 compressed。
  ⚠️ **HP60C 走 UVC 裸流**：厂商 `ascamera_node` 在本机 **1 s 后自己停流**（闭源 SDK），
  且**不要 `kill -9` 它**（会弄没 `/dev/video0`，用 `3-1.3/authorized` 0→1 复位）；
  竖屏格式 = 2560 B 头 + JPEG 640×480 + 深度 640×480×2（**0.1 mm**）+ 512 B，≈630 KB/帧。
  hub 上 640×642 只有 **9.3 fps**，1280×720 只有 0.9 fps。详见 `docs/hardware/vision.md` §3.6.2。
  ⚠️ **HP60C 裸流是左右镜像的**（2026-11-21 用户用手机相机对照确认）：绕开厂商 SDK 直接读
  UVC 拿到的是镜像画面（厂商 SDK 里应该是翻过的）。`hp60c_uvc_node` 的参数 **`mirror`
  默认 `true`**，会把**彩色 JPEG（重编码）与深度图一起翻正**，两者保持对齐；
  代价是相机节点 CPU 从 ~9% 涨到 **~21%**、单帧 34 KB → **44 KB**（q92）。
  换相机/换固件后要重新确认这一项（对着带文字的东西看字是否正常，或把手放镜头左侧看画面左右）。
  ⚠️ **MJPEG 服务只订 `compressed`，别无条件订原始图**（2026-11-21）：相机节点一旦看到
  原始图有订阅者，就要每帧解码 34 KB JPEG → 921 KB `bgr8` 再经 DDS 发出去
  （15 fps ≈ **13.8 MB/s**），实测把 `hp60c_uvc_node` 从 ~9% 顶到 **~90% CPU**、
  负载冲到 47，手机摇杆跟着"一卡一卡"。现在 `robot_mjpeg_server.py` 只在
  **3 秒没收到 compressed** 时才退回订阅原始图，压缩一到立刻退订。
  改动后实测：相机节点 CPU **9%**、MJPEG 15 fps、帧龄 23~49 ms。
  ⚠️ **禁止用 `ros2 lifecycle get/set` 做高频状态查询**（2026-11-21）：每次 CLI 要新起
  一个 DDS participant，**~2 s CPU**，而且负载一高就 10 s 超时
  （`collision_monitor` 日志会刷 `failed to send response to /get_state (timeout)`）。
  `chain_watchdog` 已改为**节点内 GetState 客户端异步查询**，CLI 只在服务不就绪时兜底且
  节流 120 s；原先每 15 s 两个 CLI 把 CPU 吃满，是摇杆卡顿与"安全链一直检查"的共同根因。
  `collision_monitor_forward` 卡在 `inactive` 时，CPU 降下来后
  `ros2 lifecycle set /collision_monitor_forward activate` 秒回（实测
  `Transitioning successful` / `active [3]`），恢复后 `/cmd_vel_auto_forward_filtered` 20 Hz 正常。
  页面手动模式**急停右侧 📷** 打开；地址由 `GET /api/devices/{id}/media` 下发（云端
  `JC_MEDIA_HOSTS`）。⚠️ **8080 不要用**（页面会重定向到 `/car/`）；**HTTPS 页面会拦 HTTP 视频**。
- 🌡 **主控温度怎么读（2026-11-24 查清）**：`/sys/class/thermal/thermal_zone*` 在本机是
  **zone0=acpitz（恒定 27.8 °C，不是 CPU）**、zone1=iwlwifi_1（无线网卡，常比 CPU 热）、
  **zone2=x86_pkg_temp（真正的 CPU 封装温度）**。`mqtt_bridge.py` 顶部还留着一张旧的
  `TEMP_PATHS`（cpu→zone0），**它没被使用**，别照着它读。
  ⚠️ `x86_pkg_temp` 会**偶发单次虚高**：同一秒连读 20 次得到 50/51/50/68/70/50 °C，
  而当时整机 CPU 稳定 25%、功率没跳 → 那 18 °C 是采样毛刺。手机上就是「温度来回跳 10 度」
  （用户 2026-11-24 报）。现在 `read_temps()` 对每个键保留最近 5 个样本发**中位数**，
  实测 25 帧跨度只剩 **1 °C**；真实升温 2~3 s 仍跟得上。有单测 `tests/test_temp_smoothing.py`。
- 启用/关闭/回滚步骤见 `docs/agent/ros-mcp-setup.md`；不要用 `FWR_ROS_MCP_MODE=full` 连实机。

### 视觉：跟随模式的人体识别（2026-11-21 新建）

- 节点 **`person_detector`**（`ros2_ws/src/fwr_bringup/fwr_bringup/person_detector.py`，
  systemd **`fwr-vision.service`**，`After=fwr-camera`）：
  - 订阅 `/robot/camera/color/image_raw/compressed`，发布 **`/robot/vision/persons`**（`std_msgs/String` JSON，**归一化坐标**）
  - 订阅 **`/robot/vision/select`**（`{"id": N}`，N<0 取消）锁定要跟随的人
  - 模型 **YOLO11n ONNX** 在 `models/yolo11n.onnx`（11 MB，onnxruntime 跑，见下）
  - 默认 8 Hz、输入 416、conf 0.35；纯逻辑在 `vision_boxes.py`（有 11 个单测）
- ⚠️ **必须用 onnxruntime，不能 cv2.dnn**：OpenCV 4.10 解析 YOLO11 的 `Concat` 节点直接报
  `Incorrect size of input array`，模型加载不起来。实测 ORT（i5-13500H）：320→13 ms、
  **416→21 ms**、640→44 ms（4 线程）。
- ⚠️ **`session.intra_op.allow_spinning = 0` 必须设**：默认自旋让线程在两次推理之间忙等，
  8 Hz 限频也压不住，实测 vision 常驻 **319% CPU**；关掉 + 2 线程后 **~55%**。
- 手机端：模式选择器里 **「跟随模式」**，画面 + 框；点框 → `GET /select?id=N`，
  框变绿显示「目标」，再点同一框或空白处取消。框由 MJPEG 服务 **`GET /boxes`** 下发
  （1 秒没新结果标 `stale`，页面显示"没在跑"而不是冻结旧框）。
- 🔒 **选中的目标必须"粘住"（2026-11-22，用户报"选好有时变绿、有时又变白要重选"）**：
  绿框原来是 `person.id === followSel` 硬比 id，而 id 由 `assign_ids` 每帧贪心重匹配，
  人走快一点/尺度一变/漏一帧就会换号 → 页面永远对不上 → 变白，只能重新点。四道修复：
  1. `vision_boxes.pick_selected(ids, last_box)`：id 换了就用**上次的框**重新认领
     （IoU 0.05 或中心距 < 0.6·max(w,h)）；画面里**只剩一个人**时直接认他。
     认领成功即把 `sel_id` 改成新号（日志 `目标 id 重新认领：1 → 7`）。
  2. 真丢了也**补发 stale 框**（最多 1 s）：`persons[]` 里带 `stale: true` 的最后位置，
     页面画**绿色虚框**「目标（短暂丢失）」——不再瞬间变白。
  3. 页面把车端的 `sel` 当**唯一权威**（`followData.sel !== followSel` 就跟随），
     车端换号后绿框立刻跟住，不用用户再点。
  4. `person_follower` **跳过 `stale` 框**：控制侧仍走自己的 `lost_stop_s 0.8 s`
     沿用上一帧逻辑，别把"丢失"伪装成"看见"。
  单测：`tests/test_person_detector.py` 的 `test_pick_selected_*`（3 例，含"都走了"）。
- 识别之外还有跟随控制（见下段「跟随控制」）；跟随走手动通道，受底盘双门控。
- ⚠️ **部署要同时落到 `ros2_ws/build/fwr_bringup/fwr_bringup/`**：实机 `ros2 run` 解析到的是
  **build 副本**（`python -c "import fwr_bringup.person_detector as m; print(m.__file__)"` 实测），
  只改 `src/` 不生效。2026-11-22 就因为只改了 `src/` 又把 import 行改漏（字符串 replace 没匹配上、没 assert），
  一框选目标就 `NameError` → 检测线程死掉 → 手机端显示"人体识别没在跑"。
  改完必须真发一次 `/robot/vision/select` 再抓 `GET :8081/boxes`，并
  `journalctl -u fwr-vision | grep -c Traceback` 确认没有新异常；`tests/test_person_detector.py`
  有静态回归（调用了的 helper 必须 import 过）。
- 单帧 JSON 里 `infer_ms` 是推理耗时，不是帧率。

**跟随控制（2026-11-21 第五版，走手动通道）**
- 节点 **`person_follower`**（`person_follower.py` + 纯逻辑 `follow_core.py`，
  systemd **`fwr-follow.service`**）：订阅 `/robot/vision/persons`、`/scan_merged`、
  `/follow/enable`；发布 **`/cmd_vel_manual`** 与 `/follow/state`。
- **为什么走 `/cmd_vel_manual`**：`base_bridge` 只认 `/cmd_vel_auto_safe`（需 nav 栈）
  和 `/cmd_vel_manual`。跟随是页面发起的手动行为，manual 通道在没起导航时也通，
  且天然受 `motion_enabled` + `manual_enabled` 双门控——**代理不用、也不能替用户开门**。
- ⚠️ **远近只看"框高占画面比例"，不要再用 `fy×1.7/h` 估米数**（2026-11-21 用户实测：
  人一走近框被画面裁掉，h 卡在 1.0，估出距离永远 ~2 m → 车"该停不停"一直往前顶）。
  `stop_box_h 0.85` 停（用户 2026-11-21 晚第三次实测：85% 才停，**框 80% 时还要在动**）、
  `resume_box_h 0.81` 才重新走
  （**迟滞只有 4%**；从 75% 上调到 85% 是因为 75% 就停车会让车"停住等人"）、
  `0.82~0.85` 按比例慢走、`≤0.30` 不追只转过去看。
- ⚠️ **速度曲线（2026-11-21 两次调快，"太慢"）**：`go_box_h` **0.62 → 0.70 → 0.82**。
  最早框 0.65~0.70（最常见的跟随距离）只有 **0.19→0.10 m/s**，体感就是"一直爬"；
  现在框 ≤0.82 全速 0.29，只有最后 3%（0.82~0.85）减速。
  **改这个值比改 `vmax` 影响大得多**。
- 🚧 **前进速度的天花板是底盘给的**：`vmax 0.29` 已经是
  `manual_max_speed_mps 0.30 − |ω|·track/2` 的极限（直行）。想真正跟上走路的人（1.0 m/s 级）
  必须把 `base_bridge.manual_max_speed_mps` 提到 **0.5~0.6**（固件超速门限 700 mm/s 兜底），
  **这是底盘安全参数，须用户批准**；提了之后同时改 `vmax` / `max_wheel_mps`。
  手机摇杆不受影响（MQTT 桥另有 `max_linear_mps 0.15` 钳位）。
- **第三版行为（2026-11-21 用户提要求后加）**：
  1. **车头一直追着框**：目标偏离画面中线 > `face_ex 0.30` → **原地转向对准**（v=0，只转），
     人从车旁走过去时车头会跟着转，不会盲目前冲；
  2. **太近就后退**：框高 ≥ `back_box_h 0.92`（人贴到车头）→ 后退 `back_vmax 0.12 m/s`，
     退到 `back_release_box_h 0.85` 才停（迟滞），**边退边按框位置转向**；
  3. **后退也避障**：`scan_metrics` 同时算**车尾走廊**（同一 ±0.22 m 走廊，`lon<0`）。
     车尾 0.60 m 内开始减速后退，**0.30 m 内（`back_hard_m`）禁止后退**，改成
     `rear_blocked` = 原地转向（用户原话"后面很靠近障碍物就原地转向就好"）；
  4. 底盘侧还有一道：`base_bridge` 的 `/base/rear_blocked`（车尾超声）为真时直接拒绝倒车指令，
     日志 `后退被车尾超声守卫拦截`——两道都过才退。
  5. **穿门框（第四版，2026-11-21 用户报"出门框卡在门框上"）**：
     **门框在车身走廊（±0.22 m）外面**（横向 0.4~0.6 m），走廊避障根本看不见它；
     人走太快时跟着框转，车角一扫就顶在门框上。现在 `passage_metrics()` 专门在
     **走廊外侧**找两侧边界：车前 1.1 m 内左右都有边界、净宽 **0.55~1.05 m** 就算"门洞"
     （走廊 1.2 m、大厅都不会误判），此时：
     - **改瞄门洞中心，不看画面的框**（`gate_pass`，v ≤ 0.18 m/s，|ω| ≤ 0.45）；
     - 中心偏 > **7 cm** 就**先原地站正**（`gate_align`，v=0），站正了再进；
     - 门洞里有人挡（框 ≥85%）→ 停，不硬挤；
     - 门洞信号消失后再保持 **0.35 s**（`gate_hold_s`）才退出，防阈值抖动。
     ⚠️ **符号**：雷达 y>0 是车**左**边，门洞中心偏左要**左转**（ω>0），
     和跟框那条 `ω = -kp·(框中心-0.5)` **相反**，写代码时别照抄（单元测试钉死了这条）。
  6. **近障碍先对准再走**：车头 0.80 m 内有东西时，"先对准再走"的门槛从 30% 收紧到
     **12%**（`align_first_ex`），避免"一边转一边前进"切内角撞门框。
  7. **门内丢人 → 先出门再找人（第五版，2026-11-21 用户要求）**：
     人比车快，出门时人先消失在画面里；以前一丢人就停车，**卡在门框中间**。
     现在门洞还在 + 目标丢失 + 记得人最后在画面哪一侧 → `gate_exit`：
     **继续按门洞中心低速（0.18 m/s）把门走完**（未对中先 `gate_align`，走廊有障碍照样停），
     最多 `gate_exit_max_s 4 s`；门洞消失（= 已出门）→ `search`：
     **原地按"最后看到人的那一侧"转身**（人在右边 → ω<0 右转，0.50 rad/s 转 2.5 s ≈72°），
     人一回到画面立刻恢复正常跟随。⚠️ 丢人时**不知道人在哪一侧**（一直居中）就不自己乱走，
     保持原来的"直接停"。
  8. **整车出门才算出门（第六版，2026-11-21 用户："前雷达出去不等于车出去，我还有后雷达"）**：
     前雷达在车头（+0.21 m），它刚过门线时**车尾还在门框里**（车长 0.46 m），
     原来一到"前雷达看不到门框"就原地转身找人 → 车尾/车角刮门框。
     现在 `door_metrics()` 用 **360° `/scan_merged`**（前+后雷达）在车身两侧
     横向 **0.28~0.80 m** 找门框点（两侧都有、净宽 0.5~1.35 m 才算"门线还在"），
     给出门线的纵向位置 `front_lon`；**只有 `front_lon < -(扫掠半径 + 0.10) = -0.415 m`**
     才算整车出去，这时才允许 `search` 原地转身。判定用 `follow_core.door_clear()`。
     ⚠️ **2026-11-22 用户第二次报「没完全出来就转向」**：旧阈值只减车尾
     （`-(0.23+0.06) = -0.29`），但原地转身时**车尾角点**扫到
     `hypot(0.23, 0.215) = 0.315 m`，比车尾还靠后 8.5 cm → 门线在 -0.30 时转身就蹭门框。
     现在阈值 = `-(hypot(body_rear_m, body_half_width_m) + door_clear_margin_m 0.10)`
     = **-0.415 m**（车尾后面约 18 cm）。另外两条配套修正：
     * `gate_exit` **不再要求 `gate_hold`**：车快出去时前雷达已经看不到门洞，
       旧写法会落到 `no_target` **停在门框里**，门线永远退不到车尾后面（死锁）。
     * `door_metrics` 返回 `None`（遮挡/超出 ±1.3 m 视场）时**不再立刻算"出去了"**，
       沿用最近 `door_lost_hold_s 0.6 s` 的测量，超时才算真出去。
     车尾还在门里时 `gate_exit` **继续沿门洞中心直出**（0.18 m/s），
     `/follow/state` 多三个字段 `door_lon` / `door_clear` / `door_thresh`。
     ⚠️ 别退化成"只看前雷达"：`passage_metrics()` 只搜 `lon ≥ 0.15` 的正前方，
     它看不到身后的门框，**必须配 `door_metrics()` 一起用**。
  决策顺序：**门口丢人先出门（车尾没出完继续直出）→ 整车出门后找人（gate_exit/search）** → 其它丢人停 →
**门前/门内先对中（gate_align/gate_pass）** → 偏太多原地转 →
  贴脸后退/车尾挡住原地转 → 到停距停 → 太远只转 → 车头走廊障碍停/减速 → 按框高给速度。
- ⚠️ **避障必须用"车身走廊"而不是角度锥**：旧写法取 ±30° 最小距离，门框在门两侧
  （0.5 m 处 ±35°、横向 0.29 m）会被当成障碍 → **人出门后车出不了门**。
  现在 `scan_metrics` 只把横向 |lat| ≤ **0.22 m**（车身半宽 0.215 + 5 mm）的点算障碍，
  走廊内 0.35 m 停、0.60 m 降速。侧向 ±30~70° 仍只看距离，用于判断"能不能转"。
- 参数（全部**跟随专用**，不动 `real_nav2.yaml`）：`vmax 0.29`、`wmax 0.70`、
  `yaw_kp 2.4`（第七版 PD；第一版 2.2 跟不住人横走、第三版 3.5 又摆，取中间）、
  `yaw_kd 0.9`、`v_accel 0.60`、`w_accel 3.0`、
  `corridor_m 0.22`、`front_hard_m 0.35`、`front_slow_m 0.60`、
  `side_clear_m 0.30`、`lost_stop 0.8`、`lost_abort 3.0`、
  `face_ex 0.30`、`back_box_h 0.92`、`back_release_box_h 0.85`、`back_vmax 0.12`、
  `back_hard_m 0.30`、`back_slow_m 0.60`、`follow_hz 20`、`smooth_alpha 0.35`、
  `max_wheel_mps 0.29`、`track_m 0.317`、`yaw_kd 0.9`、`steer_wmax 0.42`、
  `gate_look_m 1.10`、`gate_half_span_m 1.20`、`gate_min_gap_m 0.55`、`gate_wide_m 1.05`、
  `gate_align_m 0.07`、`gate_kp 2.5`、`gate_wmax 0.45`、`gate_vmax 0.18`、`gate_hold_s 0.35`、
  `align_first_m 0.80`、`align_first_ex 0.12`、
  `gate_exit_enabled True`、`gate_exit_max_s 4.0`、`search_turn_s 2.5`、`search_wmax 0.50`、
  `body_rear_m 0.23`、`body_half_width_m 0.215`、`door_clear_margin_m 0.10`、
`door_lost_hold_s 0.6`、`door_lat_min_m 0.28`、`door_lat_max_m 0.80`、
`stop_box_h 0.85`、`resume_box_h 0.81`、`go_box_h 0.82`。
- **死手开关**：页面 1 s 发一次 `/follow/enable`，节点 **3 s 收不到就停车**
  （2 s 太紧，丢一次心跳就停一下）；
  切模式/切后台/关页面都会主动发 `on=0`，并补发 2 s 零速度。
- 手机端：画面下方还有**小摇杆（104 px）**，一碰就退出自动跟随、交给人手动开。
  ⚠️ 两个摇杆**必须共用同一套 `jx/jy/joyActive`**：真正负责 20 Hz 重发的只有
  `sendTimer`（它只看这三个变量）。小摇杆若另存 `fjx/fjy`，按住不动时一条指令都不发，
  底盘 TTL(800 ms) 到点就停 → 手感"一卡一卡"（2026-11-21 用户报，手动模式正常）。
  另外 `on=0` 之后跟随节点只补发 **0.3 s** 零速度（心跳超时才补 2 s），
  否则会和刚接管的摇杆抢 `/cmd_vel_manual`。
- 🚨 **两张"底盘硬限"必须永远满足，否则整帧被丢 = 页面显示正在转向但车不动**
  （2026-11-21 用户报"显示正在转向，实际车没动"，实机日志确认 45 次
  `Rejected out-of-range manual velocity（v=0.000 w=0.900 …）`）：
  1. `|ω| ≤ 0.75` —— `base_bridge.manual_velocity()` 硬限。第二版把 `wmax` 提到 **0.9**，
     `facing`（原地转向）每次都超限被丢，所以"说在转、其实一动不动"。
     现在 `wmax 0.70`，并且 `limit_pair()` 里再夹一道 `|ω| ≤ 0.74`。
  2. **单轮包络** `|v| + |ω|·track/2 ≤ manual_max_speed_mps`（**0.30**）。
     `v=0.25` 配 `ω=0.6` 就是 `0.25+0.079=0.329 > 0.30` → **一样整帧被丢**，
     前进中一转弯就丢帧（这也是"一卡一卡"的一个来源）。
     `limit_pair()` 保 ω、按剩余预算收 v（`max_wheel_mps 0.29` 留 1 cm/s 余量），
     实测 `v=0.25,w=0.63` → 出版 `v=0.207`，不再被丢。
  ⚠️ 改 `vmax`/`wmax`/`back_vmax` 之前先算这两条；`limit_pair()` 是最后一道闸，
  **不要为了"转得更快"把它去掉**，要快就把 `manual_max_speed_mps` 提上去（需用户批准）。
- 🧭 **转向用 PD + 自动阻尼（第七版，2026-11-21 晚，用户报"直线跟随时一直左转右转"）**：
  1. **纯 P 换 PD**：`steer(err, err_dot)` = `-(kp·e + kd·err_dot)`，
     `yaw_kp 3.5 → 2.4`、新增 `yaw_kd 0.9`。阻尼项**只许收力**（夹在 `-0.7×P ~ +0.3×P`），
     否则 `-kd·err_dot` 会反号把车拽向反方向（实测 err=+0.10、dot=-0.5 会输出 +0.35）。
  2. **死区 0.05 → 0.06**，`err_dot` 用**平滑后**的框中心算（原始框每帧抖 1~2% 会把 d 项变噪声）。
  3. **跟随时 ω 上限 `steer_wmax 0.42`**（原来直接吃满 `wmax 0.70`）；偏到画面边上
     （`|err| > face_ex`，要原地对准）才允许满 0.70。
  4. **`OscGuard` 自动阻尼**（`follow_core`）：盯观测量而不是模型 —— **2 s 内 ω 变号 ≥3 次**
     就按 0.6 比例收转向增益（下限 0.35），稳住后每秒恢复 0.12。只在
     `following/obstacle_slow` 生效（`facing/gate/search` 要满速转，不能收）。
     `/follow/state` 新增 `osc_gain`、`err_dot` 便于现场判断：
     `osc_gain` 长期 <1 说明车一直在摆，这时该降 `yaw_kp` 或查底盘。
- 📚 **开源参考（2026-11-21 调研，用户要求"去网上找开源资料"）**：
  - [`Pramod-Gurav/ros2-rgbd-person-follower`](https://github.com/Pramod-Gurav/ros2-rgbd-person-follower)：
    YOLO+深度 → IoU 跟踪 → **角速度 PID + 线速度 PID**，配置 `angular_kp 0.004/px`、
    `ki 1e-4`、`kd 1e-3`、`horizontal_deadzone 30 px`、`ema_alpha 0.2`、输出 ±1.0 rad/s，
    PID 带积分限幅/输出饱和/`reset()`。**我们照它的结构**（死区+EMA+PD+输出限幅），
    但满幅等效增益比它更低（0.004×640≈2.6 rad/s vs 我们 2.4 且带阻尼）。
  - [`atom-robotics-lab/Person-Follower`](https://github.com/atom-robotics-lab/Person-Follower)：
    ROS2 差速底盘跟随仿真，同样的"框中心 → 角速度、框高/深度 → 线速度"分层。
  - [`skunal3318/ros2_person_follower`](https://github.com/skunal3318/ros2_person_follower)：
    YOLOv8 + **比例控制** + rosbridge/Three.js 面板（和我们的架构最接近）。
  - 结论：这类项目**都用"框中心误差 → PID/PD"**，没有一个用纯 P 高增益；
    共同套路 = 死区 + 输入 EMA + 输出限幅/饱和 + （可选）积分抗风。
- ⚠️ **页面"显示跟随中但车不动"的两种真凶**（2026-11-21 实机两次踩）：
  ① **底盘整帧丢弃**——`v/ω` 超 `base_bridge` 硬限（最常见的是轮距用错，见上）；
  ② **运动门/急停**——`motion_enabled` 或 `manual_enabled` 关着，指令全部被静默丢弃。
  现在节点订阅 `/odom` 自检：命令 `|v|>0.06` 持续 **1.5 s** 而里程计 `<0.03 m/s`
  → `/follow/state` 置 `nomotion: true`，页面提示行直接红字
  「⚠ 指令已发但车没动（查运动门/急停）」。**它只报警不干预**，别改成自动停车。
- ⚠️ **画面黑屏但框还在 = MJPEG 连接断了，不是识别坏了**（2026-11-21 用户报）：
  人体框走画布（数据来自 `/boxes`），视频走 `<img src=/video>` 的 multipart 长连接，
  两条连接互不影响。iPhone 切后台/切网、媒体服务重启，长连接会被静默掐断 →
  画面全黑 + 破图标，以前只能刷新页面。现在页面自带**断流自愈**：
  `onerror` 重连、每 2 s 查 `naturalWidth === 0`、`visibilitychange`/`pageshow` 回来重连、
  连续失败 3 次**重新问 `/api/devices/<id>/media`**（地址可能过期），重连间隔限 1.5 s
  （iOS 同域最多 6 个并发连接），提示行显示"画面断了，正在自动重连…（车照常跟随）"。
  摄像头页 `#camImg` 同样处理。**改视频相关代码时别把这段删掉**。
- ⚠️ **跟随"一卡一卡"的四个根因（2026-11-21 全部修掉，别再引入）**：
  1. **刷新率**：`follow_hz` 必须 **20 Hz**（和手动摇杆同频），10 Hz 对固件速度环太粗；
  2. **检测掉帧**：`person_detector` 8 Hz，YOLO 偶尔漏一帧。漏帧时节点**沿用上一帧的框**
     继续控制（最多 `lost_stop_s` 0.8 s），不加这一段就是"每漏一帧急刹一次"；
  3. **阈值抖动**：`stop 0.85 / resume 0.81` 迟滞；
  4. **框抖动**：框高与框中心做 **EMA（alpha 0.35）**，转向死区放宽到 ±5%。
- 验证：`pytest tests/test_follow_core.py`（**65 例**）+ 黑盒 `tools/sim_follow_check.py`
  （隔离 `ROS_DOMAIN_ID=42`，**22 场景**：0.29 m/s、w=-0.63（v 自动收到 0.190）、
  走廊 0.3 m 停、**框 90% 停 / 框 80% 继续跟**、宽门口正常走、检测掉帧 0.4 s 不急刹、
  **框 95% 后退 v=-0.12**、
  **车尾 0.15 m 只原地转**、**人贴边缘原地追框**、
  **正对门洞低速直穿 v=0.18**、**门洞偏左先站正 w=+0.24**、**门里人偏右不跟框转**、
  **门里丢人继续出门 v=0.18**、**出门后右转找人 w=-0.50**、
  **车尾还在门里 → 继续直出 v=0.18 不转身**、**整车出门 → 转身找人 w=-0.50**、
  **门线只到车后 0.34 m → 继续直出（旧判据会在这就转身）**、
  **目标左右摆→osc_gain 收到 <0.7**、**稳定跟随→osc_gain 恢复 >0.9**、
  **里程计不动时报 nomotion**、心跳停）。⚠️ **黑盒前先杀掉上次遗留的 42 域实例**，
  否则多个节点同时发速度，读到的还是旧值（2026-11-21 踩过）。
  **2026-11-22 08:24 实测 21 场景全 PASS**；**09:05 加 Q2 后 22 场景全 PASS**
  （S `osc_gain=0.46`、T 恢复到 1.0、Q2 `gate_exit v=0.180`），
  `fwr-follow` 08:27 重启后 `/followstate` 已带 `osc_gain/err_dot`，
  启动日志打印 `转向 PD kp 2.4/kd 0.9（跟随 ω≤0.42）`。实车跟随**仍待用户实跑**。

### 实机导航已验证参数快照（2026-11-02，四房间实景）

**已跑通完整两点导航**，以下组合可直接复用：

| 参数 | 值 | 备注 |
| --- | --- | --- |
| `inflation_radius` | **0.48** | 0.45 实测离墙 0.10 m；0.48 更居中 |
| `cost_scaling_factor` | 1.2 | 衰减慢，贴墙更贵 |
| `cost_travel_multiplier` | 8.0 | 调 20 无效果，别动 |
| `desired_linear_vel` | 0.15 | 直线速度 |
| `rotate_to_heading_angular_vel` | 0.6 | 转 90° 约 2.6 s |
| `max_angular_accel` | 4.0 | 转弯不顿挫 |
| `use_regulated_linear_velocity_scaling` | **false** | ⚠️ 开 true 会让车完全不动 |
| `front_stop` 深度 | 0.32~0.38 | 比车体前缘 0.30 留 2~8 cm |
| `dilate_cells`（map_seal） | 1 | 2 以上会把门洞挤窄导致规划失败 |
| `map_publish_period` | **0.0** | ⚠️ 必须显式写，否则 30 Hz 重发冲垮 AMCL |
| `track_unknown_space` | false | true 会让全部路径规划失败 |
| BT `number_of_retries` | 6 | 2 时 0.6 s 就放弃 |
| `auto_initial_pose.max_score_m` | 0.15 | 0.10 会拒绝 0.127 m 的正常匹配 |

**控制 / 行为树（v1.0 定版）**
- `angular_dist_threshold` **0.6**（RotationShim 转向阈值；0.35 时门口
  每个弯都停下原地转，实测 150 s 内 4 次线速度归零 → 门口"特别慢"）
- `use_velocity_scaled_lookahead_dist` **true**
- BT `RateController` **0.5 Hz**（⚠️ 1 Hz 时远处目标要 1040 ms 规划，
  刚算完就被取消 → "点开始就结束"）
- `bt_loop_duration` **50 ms**（100 Hz 会被饿死，报 `tick rate exceeded`）
- `xy_goal_tolerance` **0.4**（规划器会把落在膨胀区的终点吸附 0.34 m）
- 选择器：角速度符号容差 **0.05 rad/s**；方向变化**不清空**回执缓存
  （这两条是"一卡一卡"的两个根因，见 AGENT_LOG）

**速度平滑器（2026-11-03 定版，用户确认"好点了"）**
- `max_velocity = [0.15, 0.0, 1.0]` —— 必须与 controller 的 `desired_linear_vel`
  一致。原来写 0.12，把指令**永久截断**，车从来没跑到过 0.15。
- `max_accel = [0.35, 0.0, 4.0]` —— 上限从 0.12 提到 0.15 后，控制器的
  ±0.02 波动会原样传到电机（直线"一卡一卡"）。把加速度限制降到 0.35
  等于补回低通滤波，速度不变但变化更绵。

> ⚠️ **未解之谜（留给后续）**：`/cmd_vel` 均值 0.142，而 `/odom`
> 均值 0.231（高 63%）、标准差 0.11（是指令的 5 倍）。
> 但另一次 1.5 m 直线测试里 odom 只报 1.264 m（偏低）。
> 两者矛盾，说明**里程计标度可能有问题**（轮径/每圈计数/轮距）。
> 影响：Nav2 的"走了多远"判断不准，goal 容差可能提前触发。
> **建议专门做一次卷尺标定**（直行 2 m，对比 odom 与实测）。

**底盘速度补偿（v1.2 定版）**
- `base_bridge.wheel_speed_scale = 0.66` —— 固件速度环**稳态偏高 1.5 倍**
  （下发 141 mm/s，固件自己也报 214 mm/s，说明它知道但积分下限为 0 无法下修）。
  在 ROS 侧按固定比例补偿；**v 与 ω 同比缩放，转弯半径不变**。
  ⚠️ **固件重新标定后必须把它改回 1.0**。
- ⚠️ **不要单独放开固件积分的负下限**：试过 `clamp(-0.30, 0.30)`，
  速度环收敛了（141→143.7）但**前馈仍偏高**，两者抢权导致极限环振荡
  （实测车速 0↔0.46 m/s、80 s 内 12 次停走）。要改就**同时**把前馈
  `0.22+0.0035v` 降到 ~`0.22+0.0022v`，并做台架测速。

**碰撞监控 / 代价地图（2026-11-02 最终）**
- `front_stop` 深度 **0.32~0.38**（半宽 0.24，3 点触发）—— 浅于 0.30 会撞
- `front_slow` 横向 **±0.22**、深度 **0.30~0.50**、`max_points 3`、`slowdown_ratio 0.8`
  （深度 0.75 时穿过门会扫到"对面门框"→ 持续扣 20% 速度）
  （±0.30 / 1 点时会因侧墙和门框误触发，一次跑动 7 次）
- `local_costmap.plugins = [obstacle_layer, inflation_layer]`
- `footprint_padding 0.02`

> ⚠️ **这四个"手感"参数一次只改一个**：`angular_dist_threshold`、
> `use_velocity_scaled_lookahead_dist`、`inflation_radius`、`front_stop.max_points`。
> 2026-11-02 一起改（0.35→0.7、true→false、0.48→0.50、3→6）后出现
> **起步即原地来回转**；全部回退后立刻恢复正常。改一个、跑一次，再动下一个。

## 3.97 🔒 参数冻结规则（2026-11-04 用户明确要求）

**用户原话：「把这些参数都记下来，不要随便修改，后面改的时候一定要经过我批准」**

### 硬规则（违反即为事故）
1. 下列参数**未经用户在当次会话里明确批准，一律不许改**（包括"顺手优化"）。
2. 需要改时：先说清**改哪个、从多少改到多少、为什么、预期效果、如何验证**，
   等用户确认后再动手。
3. 一次只改一个，改完必须实机验证并把数字写进 `docs/AGENT_LOG.md`。
4. 有护栏测试守着的值（见各节注释）**连提议都要先说明为什么护栏允许**。

### 冻结清单（v1.4 实际运行值）
**导航 `config/real_nav2.yaml`**
| 参数 | 值 |
| --- | --- |
| `FollowPath.desired_linear_vel` | 0.20 |
| `FollowPath.lookahead_dist` / `min` / `max` | **0.55** / 0.40 / 0.85 |
| `FollowPath.angular_dist_threshold` | 0.6 |
| `FollowPath.rotate_to_heading_angular_vel` | **0.55** |
| `FollowPath.max_angular_accel` | 4.0 |
| `FollowPath.use_regulated_linear_velocity_scaling` | false |
| `FollowPath.use_cost_regulated_linear_velocity_scaling` | **false** |
| `inflation_radius`（全局/局部） | 0.48 |
| `cost_scaling_factor` | 1.2 |
| `footprint_padding` | 0.02 |
| `xy_goal_tolerance` | 0.4 |
| velocity_smoother `max_velocity` / `max_accel` | [0.20,0,1.0] / [0.35,0,4.0] |
| BT `RateController` | 0.5 Hz |

**碰撞监控 `config/real_collision_monitor_forward.yaml`**
| 参数 | 值 |
| --- | --- |
| `front_stop.points` | [0.32,0.22, 0.38,0.22, 0.38,-0.22, 0.32,-0.22]（半宽 **0.22**，护栏下限）|
| `front_slow.points` | [0.30,0.22, 0.50,0.22, 0.50,-0.22, 0.30,-0.22] |
| `stop_pub_timeout` | **600.0**（2026-11-20 批准 18→600；护栏允许 17.0 ≤ value ≤ 600.0）|
| `source_timeout` | 0.6 |

**底盘 `base_bridge`**
| 参数 | 值 |
| --- | --- |
| `wheel_speed_scale` | **0.66** |
| `sensor_timeout_s` | 0.5 |

**探索器 `frontier_explorer`（导航侧）**
| 参数 | 值 |
| --- | --- |
| `stuck_cancel_s` | 2.5 |
| `max_stuck_retries` | 2 |

**建图侧（`real_auto_mapping.launch.py` 传给 room_explorer 的）**
| 参数 | 值 |
| --- | --- |
| `escape_after_failures` | 9999 |
| `replan_s` | 1.5 |
| `use_room_gate` | False |

## 3.99 🔐 部署 / 启动必须逐次经用户同意（2026-11-24 用户明确要求）

**用户原话：「下次每次部署和开始的时候都要经过我同意」。**

### 硬规则
1. **部署**：任何把代码/页面/配置送到车端或云端的动作（`scp` / `rsync` / `cp` 到
   实机或 ECS、写入 `/home/boat_control/`、`/opt/jc/web/`、覆盖 systemd 单元等），
   **先在对话里说清「改哪个文件、送到哪里、影响什么」，等用户回「可以」再执行**。
   改动前的备份照旧要做（`*.bak.<时间戳>`）。
2. **启动**：任何 `systemctl start/restart/enable/stop`、`ros2 launch` / `ros2 run`、
   拉起服务或节点的动作，**同样先经用户同意**（含"顺手重启一下"）。
   读操作（`status` / `journalctl` / `ros2 topic echo` / `param get`）不受限。
3. **代码编写/测试**不算部署，但**"开始某项任务"前要先说明方案**并得到用户点头
   （结束语里明确问"现在开始吗"）。
4. git 仓库内的 **commit / push 仍授权**（本会话既有授权），它不等于部署到实机。
5. 违反后果：用户在现场看不到变化原因、或车在无人看护时被重启。
   **一次只做用户点头的那一项，做完汇报，再问下一项。**

## 3.975 🧹 全局代价地图：障碍层**保留** + 空闲时周期清空（2026-11-07 定版）

> 2026-11-05 曾把全局障碍层**删掉**（治"幽灵障碍"），结果**路上新出现的障碍不会被绕行**：
> 规划器看不到动态障碍，车只会被 `front_stop` 停住干等。**已回退，不要再去掉。**

现在 `global_costmap.plugins = [static_layer, obstacle_layer, inflation_layer]`。

### 污染物怎么治：周期清空
`frontier_explorer` 的 `costmap_clear_period_s`（默认 **10 s**）在两个代价地图上调用
`ClearEntireCostmap`，**只在空闲时执行**（导航中清空会在重规划那一帧把动态障碍一起清掉）。
清空后下一帧扫描立刻重新标记当前真实障碍，只有"车身后方永远扫不到的陈旧标记"被彻底清除。

### 实测（2026-11-05）
- 致命格：真墙 4007 / 墙旁 1 格 1 / **幽灵 1（≈0%）**
- 判断幽灵必须用 **`cost == 100`**；用 `>= 65` 会把**膨胀区**误算成幽灵（踩过两次）。

## 3.976 🧭 多点导航的四个"提前结束"陷阱（2026-11-05~07，都已修）

`frontier_explorer._start_multi_point_goal` 的分支规则（**改这里之前先读完本段**）：

| 陷阱 | 现象 | 现在的处理 |
| --- | --- | --- |
| 终点离**车**近 | goal checker 立刻判到达 | `tail_gap < fixed_route_tail_gap_m`（2.5 m）→ **逐段执行** |
| 终点离**路线起点**近（闭环）| 车去起点路上路过终点 → 判到达 → 后面点全跳过 | `loop_gap < 2.5 m` 同样改**逐段执行** |
| 路线首点就在车身上 | through-poses 零长度首段 → 车原地打转 | `route_skip_near_m`（0.6 m）剔除，**但终点永不剔除** |
| 规划失败（如 `Starting point in lethal space!`）| 行为树重试 6 次后 abort → 整条路线结束 | `_finished` 里清空代价地图并**重派同一段**，最多 `max_stuck_retries`（2）次 |

### 相关参数（v1.7 实际值）
| 参数 | 值 | 说明 |
| --- | --- | --- |
| `fixed_route_tail_gap_m` | 2.5 | 终点距车/距起点小于它 → 逐段执行 |
| `route_skip_near_m` | 0.6 | 剔除车已站住的**非终点**路线点 |
| `costmap_clear_period_s` | 10.0 | 空闲时周期清空代价地图 |
| `reverse_pause_enabled` | True | 后退让位期间暂停任务（不中止） |
| `max_reverse_episodes_per_leg` | 2 | 同一点让位超过它 → 跳过该点 |
| `max_stuck_retries` | 2 | 导航失败后清代价地图重试次数 |

### "停止/继续"的正确语义（2026-11-07）
- 停止只**挂起**任务：`_fixed_goals` / `_multi_points` 和段索引都保留。
- 继续走 `/explore/resume`，**按节点记住的进度**接着走本该走的那一段；
  面板只有在节点说"没有可继续的路线"时才按路线重发。
- 一次穿过（through-poses）模式继续时要**重置派发计数**（`sm.reset_dispatched()`），
  否则状态机认为"已走完"立刻停。

## 3.977 🧍 后退让位（trail_reverser）状态：**默认开启**（2026-11-20 起）

- `real_navigation.launch.py` 的 `trail_reverser` 默认值 = **true**，手机端「开始导航」
  起的栈里 `/trail_reverser` 会运行（2026-11-20 现场核对：节点在、`enabled=True`）。
- 触发条件（全部满足）：功能启用 / 车尾不近障 / **左右两侧都 < 0.5 m**（用 `max` 判，
  原来左右混一起取 min 是错的）/ 前方被挡 ≥1 s / 轨迹 ≥0.5 m / 冷却 5 s。
  - **"被挡"的判据（2026-11-20 修）**：除"过滤后速度≈0"外，新增
    `no_forward`——原始指令想前进（`raw.linear.x > 0.02`）但过滤后线速度≈0
    （方向安全选择器把"前进+转向"改判成纯原地转）也计为被挡。
    此前这种情况 `blocked_for_s` 永不累计 → 车头顶墙只会左右转、不后退。
- 后退 0.08 m/s，沿记录轨迹；任意一侧 ≥0.9 m、车尾 <15 cm、1.2 m、10 s 任一满足即结束。
- **收工余量**：`backup_front_clear_m`（默认 **0.60 m**）——车头净空 ≥
  max(`front_need_m` 0.45, 0.60) 才判定"能转了"收工；此前只留 4 cm（0.45），
  退完立刻又贴墙。
- 与任务的关系：发布 `/reverse/active`；explorer 看到后**暂停**任务，结束再恢复；
  同一点让位 >2 次 → 跳过该点（逐段：`skip_leg`；through-poses：删最近点）。
- ⚠️ 该链路**只有单元测试与独立域模拟**（`tools/sim_reverse_chain_*.py`，需 `ROS_DOMAIN_ID≠0`），
  **尚未实机验证**。

## 3.974 🔧 建图匹配坐标翻转（2026-11-05，严重 bug）

`auto_seed_core.score_pose` 用 `height-1-(y-origin)/res`（图片行序）索引 OccupancyGrid，
把地图**上下镜像** → 自动播种把车放到镜像位置（每次启动落在相距 7~13 m 的点）。

- 证据：同一扫描/地图，不翻转残差 0.120 m @ (-0.10,-0.80)（= 用户手动摆的位置）；
  翻转残差 0.123 m @ (6.20,-9.80)（= 播种结果）。
- 修复后：`match 0.009 m -> (-0.14,-0.91,-75°)`，**贴墙率 98%**（修复前 1~3%）。
- 新增检查：贴墙率 ≥60%、未知区 ≤20%、位置必须在空闲区且有 10 cm 净空；
  任一项不过就**拒绝播种**，等人工 2D Pose Estimate（手动位姿现在不会被覆盖）。
- 回归测试：`test_auto_seed_convention.py` 钉死行序约定。

## 3.96 ⭐ v1.4 定版（2026-11-04，解决"过门慢 / 门口卡住"）

v1.3 之后的改动（都已实车验证）：

### 参数
| 参数 | v1.3 | **v1.4** | 原因 |
| --- | --- | --- | --- |
| `front_stop.points` 半宽 | 0.24 | **0.22** | 0.24 比车体(0.215)宽 2.5 cm，门框落进矩形角点 → 42 次停车 |
| `FollowPath.lookahead_dist` | 0.60 | **0.55** | 前视太远切内弯、贴门框 |
| `FollowPath.rotate_to_heading_angular_vel` | 0.6 | **0.55** | 掉头过冲差点撞门框 |
| `FollowPath.use_cost_regulated_linear_velocity_scaling` | true(默认) | **false** | **过门慢的主因**：车身两侧同时落在门框膨胀区 → 代价饱和 → 降速 |
| `stop_pub_timeout` | 18.0 | **18.0（不变）** | 试过 2.0 被两条护栏测试否决（需 ≥17 s 让 Nav2 进度检查放弃目标） |

### 代码
- `frontier_explorer`：监控持续压零 **2.5 s 且停车框确认为空**（数 /scan 落在框内的点数）
  → 取消当前目标并**自动重试**（`max_stuck_retries=2`）。
  真障碍不会被误判取消；只有 `stop_pub_timeout` 残留才重试。

### 实测（v1.4，一趟导航）
```
front_stop 触发        0 次        （v1.3 前 42 次）
行进速度均值           0.196 m/s   （上限 0.20，门口不再掉速）
侧向净空 10%分位       0.43 m      中位 0.94 m
卡住/重试              0 次
目标                   2 次全部 succeeded
```

### 多点导航（2026-11-04 实车验证通过 ✅）
- 闭环路线（终点离车 <1 m）→ 自动改**逐段执行**，实测 6 段全部走完
  （日志：`终点距车仅 0.35 m（闭环路线）→ 改为逐段执行 6 段`）
- 终点 >1 m 的普通路线 → `NavigateThroughPoses` 一次穿过，途经点不停车
- 运行中卡住 0 次、重试 0 次

### 明天的注意点
- 改这些值之前先看本节：`front_stop` 半宽**不能小于 0.22**（有护栏测试守着，
  是车体 0.215 + 5 mm）；`stop_pub_timeout` **不能小于 17**（两条测试守着）。
- `use_cost_regulated_linear_velocity_scaling=false` 意味着**门口不再自动降速**，
  贴墙/贴门的保护只剩 `front_stop` 和选择器，改速度相关参数时要记得这一点。

## 3.95 ⭐ v1.3 冻结快照（2026-11-03 收工，**用户要求：先不要改动**）

明天开工先读这一节。**除用户明确要求外，下面这些值一个都不要动。**

### 导航（`config/real_nav2.yaml`，实际运行值）
| 参数 | 值 |
| --- | --- |
| `FollowPath.desired_linear_vel` | **0.20** |
| `FollowPath.angular_dist_threshold` | **0.6** |
| `FollowPath.lookahead_dist` / `min` / `max` | **0.60 / 0.40 / 0.85** |
| `use_velocity_scaled_lookahead_dist` | true |
| `inflation_radius`（全局/局部） | **0.48** |
| `footprint_padding` | **0.02** |
| `cost_scaling_factor` | 1.2 |
| `xy_goal_tolerance` | 0.4 |
| velocity_smoother `max_velocity` | **[0.20, 0.0, 1.0]** |
| velocity_smoother `max_accel` | **[0.35, 0.0, 4.0]** |
| BT `RateController` | **0.5 Hz** |

### 底盘
- `base_bridge.wheel_speed_scale` = **0.66**

### 今天修好并冻结的行为
- 安全链：左右扇区不再取同一个值；ω 符号抖动只归零转向、保留前进
- 单点 / 两点 / **多点导航**（多点用 `NavigateThroughPoses`，途经点不停车；
  终点距车 <1 m 的闭环自动改逐段执行）
- 多点预览画出穿过所有途经点的完整折线

### 明天的任务（用户指定）
**解决"靠近门口速度慢"**。已知相关信息：
- `front_slow` 参数（横向 ±0.22、深度 0.30~0.50、`slowdown_ratio 0.8`）
- `approach_velocity_scaling_dist`（默认 0.6，**未显式设置**）
- RPP 的曲率限速（转弯时 0.06~0.15）
- 排查顺序：先看 `/cmd_vel_nav` 在门口的实际数值 vs `/cmd_vel`，确认是哪一层在降速
  （控制器自身 / 平滑器 / collision_monitor 的 slowdown），再动参数。**一次只改一个。**

### ⚠️ 建图调参禁止修改导航共用文件（2026-11-03 血的教训）

`real_nav2.yaml` 和 `real_navigate_to_pose.xml` 是**导航与建图共用**的。
2026-11-03 为了建图效率改了下面这些值，**导航立刻退化**（直线左右摆、门口转不过去）：

| 参数 | 导航（v1.2，正确值） | 建图时曾改成 | 后果 |
| --- | --- | --- | --- |
| `FollowPath.angular_dist_threshold` | **0.6** | 3.1416 | 关掉原地对准 → 门口转不过去 |
| `FollowPath.lookahead_dist` | **0.60** | 0.45 | 前视太短 → 直线左右摆 |
| `FollowPath.min_lookahead_dist` | **0.40** | 0.32 | 同上 |
| `FollowPath.approach_velocity_scaling_dist` | 默认 0.6 | 0.25 | 临近终点减速区变化 |
| BT `RateController` | **0.5 Hz** | 0.25 Hz | 重规划变慢 |

**当前状态**：以上全部已恢复 v1.2（commit `4c5a70c`）。
**规则**：以后建图调参只能在**建图专用**的文件/launch 里做，
或者干脆接受建图慢一点。**不要再动这两个共用文件。**

**自动建图的坑（2026-11-03 全部踩过）**
- **建图必须用 `real_mapping.rviz`**（launch 里已改好）：`real_localization.rviz`
  里的 Map Edit 面板会持续重发旧 `/map`，与 slam 的新地图交替刷屏 → 看着像"地图不长"。
- **`use_room_gate: False`**：房间门控会把走廊当成一个房间，只要它还有前沿就永不
  进别的房间（实测出房间后困在走廊）。关掉用全局前沿。
- **改完 launch 参数必须 `ros2 param get /<node> <param>` 核对**：曾把参数加到
  相邻的 `map_free_around_robot` 上，探索器一直用默认值，优化实际没生效。
- **效率参数（2026-11-03 实测最优）**：`angular_dist_threshold 3.1416`（不平移转向）、
  `approach_velocity_scaling_dist 0.25`、BT `RateController 0.25 Hz`、
  探索器 `replan_s 1.5`、目标超时 15 s。实测前进 45.6% / 静止 7.8% / 地图 +8529(80 s)。
- **不要靠加 inflation/footprint_padding 让过门更居中**：0.50 / 0.05 都会让墙边
  候选落点变成 cost>=99 → 探索器连续拉黑 → 车完全不走（实测）。保持 0.48 / 0.02。
- **脱困有两条触发路径**：一条受 `escape_after_failures`，另一条（连续 3 次不可通行）
  曾经是**硬编码**。脱困 = `_set_manual(True)` = 自动链路被门控清零 = 车彻底停住。
  现在两条都服从同一参数。
- **死胡同判定要数 `cost < 99` 的格子**，不能数 `cost == 0`：膨胀 0.48 让墙边
  代价都 >0，数 0 会把几乎所有前沿误判成死胡同（实测同一目标连续 73 次被拒）。
- **墙脚目标落点搜索半径 1.2 m，且找不到要回退前沿点本身**（不能返回 None）：
  返回 None 会导致"连续拉黑 → 只转不走"。修复后原地转 66% → 3.2%。
- **墙脚目标落点搜索半径 0.8 m**（原 0.4 m）：墙脚附近全被膨胀覆盖，0.4 m 内
  找不到合法落点 → 目标被拉黑 → 连续失败进脱困 → 左右转循环。
- **前沿不能太近**：兜底分支已要求 ≥0.5 m。以前会派 0.2 m 的前沿 → Nav2 认为已到点、
  `/cmd_vel_nav` 恒 0 → 车原地不动、探索器反复派同一点。
- **起步要开阔**：kickstart 在 `front < 0.45 m` 时会反复 blocked/orient，
  走不完就等 30 s 超时；这期间 `manual_enabled=True`，**自动链路全被门控清零**。
- **建图前查雷达**：`ros2 topic hz /scan` 应 ≈10 Hz。残留 `sllidar` 进程会占着
  `/dev/ttyUSB0`，新雷达起不来且**不报错** → ready=False、slam 无输入、TF 外推失败。
  用 `kill -9 <旧 PID>` 清掉。
- **超声不要改成"只在倒车时轮询"**：`rear_guard` 无数据会 fail-safe 报车尾被挡，
  探索器脱困永远失败 → `manual_enabled` 卡 True → 自动链路被门控清零。
- **节点"启动后一行日志都没有"= 大概率没挂上定时器**。已有回归测试
  `test_auto_mapping_nodes_wired` 守住这条。

**安全链两个已修 bug（2026-11-03，别再引入）**
- `directional_safety_selector._sector_min`：`if side>0 / else` 两块代码**曾经完全一样**，
  导致左右永远取同一值、"往空旷一侧转"永不触发。左前 = 150~180°，右前 = -180~-150°。
- `directional_safety_core`：ω 符号与监控回执不符时**整帧归零**会导致直线顿挫。
  现在前进行驶只归零转向、保留前进；纯旋转仍整帧拒绝（有单测守护，别改）。

**⚠️ 局部代价地图不要启用 `range_layer`**：它等不到超声数据时会把代价地图
更新拖住 → 控制器拿不到新鲜地图 → `computeVelocityCommands` 失败 →
`[follow_path] Aborting handle` 每秒一次、`/cmd_vel_nav` 恒 0。
现象是"短目标能到、长目标必卡"。倒车安全由 `rear_guard` 独立保障。

**Map Edit 面板**：橡皮工具已加发布去重 + 4 Hz 限速（原来每帧重发 `/map`，
30 Hz 冲垮 AMCL，是"车完全不动"的总根源）。

## 3.9 血泪教训（2026-11-01，务必遵守）

1. **"规划穿墙"先查数据源，别查算法**
   那天把规划器、代价地图、膨胀、旋转逻辑查了个遍，真凶是 `map_seal` 输出的
   `/map_sealed` 与 `/map` **逐点错位**（151 个墙点 0 个对得上）——
   代价地图里的墙根本不在看到的位置。
   **排查顺序**：`/map` → `/map_sealed` → `/global_costmap/costmap`
   三级**逐点**比对（同一世界坐标上三者的值必须一致），再怀疑算法。

2. **任何"坐标搬运/缩放/裁剪"代码必须逐点校验**
   固定画布、图像缩放、坐标系变换这类代码，**只看尺寸对是不够的**。
   要取真实特征点（墙点）验证落位。写完后立刻 A/B 比对。

3. **RViz 里的线不等于实时数据**
   Path 显示会一直保留最后一次收到的消息。判断"规划对不对"要抓话题
   （`/plan`、`/two_point_preview_path`）的**实时内容**，不是看画面。

4. **改安全/代价参数必须 A/B 回归**
   例：`track_unknown_space: true` 一次就让 3 组路径**全部规划失败**。
   改任何影响规划的参数，先记录"改之前能成功的 3 组起终点"，改完重跑对比。

5. **新增功能不得破坏旧功能**（用户反复强调）
   加新按钮/新逻辑时**复用已验证的函数**（如 `loadAndPublishMap`），
   不要另写一套并行实现。每次改完必须回归：旧路径至少跑一次。

## 3.98 🛡 防"车不动"的两道自检（2026-11-04）

2026-11-04 连续两次因为**低级代码错误**导致"点开始导航但车完全不动"：
新增回调方法没插进文件、新增字段没在 `__init__` 赋值。都补了自动检查。

### ① 静态守护（每次跑测试都会执行）
`test_nodes_attributes_defined.py`：AST 扫描所有节点类，
**读取了从未赋值过的 `self.xxx`** 就报错。这类 bug 运行时表现就是节点崩、车不动。
```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
PYTHONPATH=ros2_ws/src/fwr_bringup .venv/bin/python -m pytest ros2_ws/src/fwr_bringup/test -q
```

### ② 运行自检（每次启动栈、开导航之前跑）
```bash
bash tools/check_stack.sh /tmp/fwr_runs/<本次日志>.log
```
检查六项：进程存活 / 节点在 ROS 图里 / 三个导航服务在位 /
**冻结参数是否被意外改动** / 雷达频率 / launch 日志有无 `process has died`。
全绿才开导航；不绿会列出具体哪一项。

## 4. 验证矩阵（先跑最小集，最后再全量）

| 改动类型 | 最小验证 |
| --- | --- |
| 纯 Python 核心/协议/几何 | `PYTHONPATH=ros2_ws/src/fwr_bringup .venv/bin/python -m pytest -q ros2_ws/src/fwr_bringup/test/test_<area>.py` |
| 影响多个节点/launch | 先跑对应 test 文件，再 `source /opt/ros/humble/setup.bash && .venv/bin/python -m pytest -q tests ros2_ws/src/fwr_bringup/test` |
| ROS 包/接口/launch | `cd ros2_ws && colcon build --packages-select fwr_interfaces fwr_bringup --symlink-install` |
| launch 语法 | `.venv/bin/python -m py_compile <launch.py>`；必要时 `ros2 launch <pkg> <file> --show-args` |
| ESP32-S3 固件 | 主机测试与 `idf.py build` 见 `firmware/esp32s3_base_control/README.md`；烧录前断开电机 12 V |
| 仿真 | 用隔离 `ROS_DOMAIN_ID` + 独立日志，例：`/tmp/fwr_*.log`；不要占用实机话题 |

测试证据必须写清：**命令 + 结果 + 日志路径**。只跑过测试不算“实机已验证”。

## 5. 安全红线

- **电量低于 21.6 V（单节 3.60 V）不要跑自动导航/长距离**：2026-11-22 08:40
  就是低电整机掉电（`last` 标 crash、无关机序列，掉电前 JSY 表 USB 先消失、后雷达串口超时）。
  低电会把「雷达崩 / 面板 `--` / 安全链卡 inactive」全搅在一起，排查时先看 `journalctl --list-boots`。
- **未经用户在当前会话明确同意，不向实机发运动命令**，不直接 `ros2 topic pub /cmd_vel`，不把 `motion_enabled` 改成 true。
- 不自动 `git commit` / `git push`；需要时先问用户。
- 不把 API key、密码写进仓库或配置文件；使用 `FWR_LLM_API_KEY` 等环境变量。
- 烧录/带电测试：先断电机 12 V、车轮悬空、可随时断电。软件停止 ≠ 硬件急停。
- 测试不依赖真实硬件；硬件结论必须单独标注“已验证 / 未验证”。

## 6. 省 token / 防重复（硬规则）

- **一个事实只写一处**：长期规则写本文件；会话结果写 `AGENT_LOG.md`；计划进度写 `progress.md`。不要三处复制。
- **改动即文档（用户 2026-11-21 明确要求）**：每次改完代码/参数/硬件，**当场**同步文档，不要攒到收尾。
  长期有效的事实（硬件、接口、冻结值、已验证行为）→ 本文件；当天的过程与证据 → `docs/AGENT_LOG.md`。
- **先搜后做**：动手前用 `grep` 搜代码、日志和计划，确认没做过；不要凭印象重跑一遍排查。
- **输出要小**：`head` / `tail`、`--once`、`-l`、`wc -l`、`timeout`；大输出先落盘再 `grep`，不要把整个日志贴进上下文。
- **独立调用并行**：一个消息里批量发互不依赖的读取/搜索；大范围探索交给 subagent，只让它回传 ≤20 行结论。
- **不在会话中途重写本文件**：会改变提示前缀、降低 KV 缓存命中；要么开工前改，要么收尾时改。
- **阶段边界压缩**：长任务在完成一个阶段后用 `/compact` 或开新会话；不要把整段历史拖到上下文腐烂。
- **最小验证**：改什么跑什么；全量测试只在交接前跑一次，不反复刷。
- **失败也记**：试过但没用的方案写进日志，避免下个会话再踩。

## 7. 会话收尾（必须）

在 `docs/AGENT_LOG.md` 末尾追加一条（只追加、不删除历史，时间用 Asia/Shanghai）：

```markdown
## YYYY-MM-DD HH:MM · 一句话标题

### 会话目标
### 仓库改动（文件 / 新增 / 修改 / 删除）
### 仓库外改动（环境变量、系统配置、设备）
### 已验证（命令 + 结果 + 日志路径）
### 未验证 / 遗留问题
### 当前运行状态（进程、PID、端口）
### 下一步
```

同时更新对应 `progress.md`；只有**长期有效的事实**才回写本文件。下一位接手者应先读最后 2–3 条日志，不要假设“上次说的”已经落地。

更细的按需流程见项目 skill：`.agents/skills/fwr-workflow/SKILL.md`。
调研依据与插件清单见 `docs/agent/token-efficient-workflow.md`。
