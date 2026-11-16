# 手机远程控制系统 v1.0（2026-11-15）

> 状态：可用。手机浏览器（Android / iOS）打开 `http://8.163.69.109:8080/car.html`
> 即可手动遥控、看激光/地图/电量、点选航点导航。

## 1. 架构

```
手机浏览器 ──HTTPS/WS──► 阿里云 ECS 8.163.69.109 ──MQTT──► 小车 car-001
（car.html）              nginx :80 / :8443               mqtt_bridge（systemd）
                          FastAPI :8444 (systemd)         /cmd_vel_manual
                          EMQX   :1883                     /explore/*
                          静态页 :8080（boat-web）         /plan /scan /local_plan
```

- **用户不直连 MQTT**：页面只连 `FastAPI` 的 WebSocket，由服务端转发到 EMQX。
- **车辆主动外连**：车端 MQTT 客户端连接云端 1883，无需家里有公网 IP。
- 域名 `jccontrol.xyz` 已解析到该 ECS，但**尚未 ICP 备案**，80/443 被阿里云拦截；
  当前用 `http://IP:8080/car.html`（非 443 端口不触发 HTTPS 强制升级）。

## 2. 云端组件

| 服务 | 端口 | 管理 |
| --- | --- | --- |
| nginx | 80 / 443 | `nginx -s reload` |
| FastAPI（账号/设备/WS/地图/路径） | 127.0.0.1:8444 | `systemctl restart jc-api` |
| EMQX（MQTT Broker） | 1883 / 8083(WS) / 18083 | `systemctl restart emqx` |
| 静态页 car.html | 8080 | `/home/boat_control/car.html` |
| SQLite | `/opt/jc/data/jc.db` | users / devices |

FastAPI 关键环境变量（`/etc/systemd/system/jc-api.service`）：
`JC_DB`、`JC_SECRET`、`JC_BIND_CODE=123456`、`JC_MQTT_HOST=127.0.0.1`、
`JC_MAX_LINEAR=0.30`、`JC_MAX_ANGULAR=0.6`。

### HTTP / WS 端点

| 端点 | 说明 |
| --- | --- |
| `POST /api/register` | 邮箱 + 密码（≥6 位）|
| `POST /api/login` | 返回 JWT |
| `POST /api/devices/bind` | `{device_id, code}`，绑定码正确即可转移归属 |
| `GET /api/devices` | 我的设备 + 最近状态 |
| `GET /api/devices/{id}/state` | 最近状态缓存 |
| `GET /api/devices/{id}/map` | 最近一次导航用的地图（gzip+base64 PGM + 元数据）|
| `WS /ws/control/{device_id}?token=` | 控制：`{linear,angular}` / `{type:'stop'}` / `{type:'goal',points}` / `{type:'preview',points}` / `{type:'nav',action}` |

`nav.action` 支持：`stop`（`/explore/stop`）、`resume`、`manual`（开手动门）、
`relocalize`（`/localization/reseed` 一键播种）。

## 3. 车端 `mqtt_bridge`（`ros2_ws/src/fwr_bringup/fwr_bringup/mqtt_bridge.py`）

systemd 服务 `fwr-mqtt-bridge.service`，开机自启。

| 方向 | MQTT 主题 | 内容 |
| --- | --- | --- |
| 下行 | `robot/car-001/cmd` | 手动速度（含钳位、TTL、seq 去重、0.5s deadman）|
| 下行 | `robot/car-001/goal` | `{points:[[x,y],...]}` → 自动开门 + `arm_auto` + `/explore/start_fixed_goal`(1点) / `start_multi_point_goal`(多点) |
| 下行 | `robot/car-001/preview` | 只算路径不执行：1 点 `ComputePathToPose`，多点 `ComputePathThroughPoses` |
| 下行 | `robot/car-001/nav` | stop / resume / manual / relocalize |
| 上行 | `robot/car-001/state` | 10 Hz：电压/电流/电量/位姿/nav 状态/fault/armed |
| 上行 | `robot/car-001/scan` | 约 3.6 Hz，抽稀到约 100 点（map 坐标系，厘米）|
| 上行 | `robot/car-001/path` | 全局路径（`/plan`，仅在规划时）|
| 上行 | `robot/car-001/local_plan` | 局部避障轨迹（`/local_plan`）|
| 上行 | `robot/car-001/preview_path` | 预览路径结果 |
| 上行 | `robot/car-001/map` | 启动时上传一次（retained，gzip+base64）|
| 上行 | `robot/car-001/goal_result` | 目标/导航指令的接受结果 |
| 在线 | `robot/car-001/online` | LWT：online/offline |

关键参数：`state_hz=10`、`scan_hz=5`、`scan_points=180`、`max_linear_mps=0.30`、
`max_angular_rps=0.6`、`cmd_timeout_s=0.5`。

## 4. 页面功能（car.html）

- 顶部：刷新（强制重载 `?v=时间戳`）、模式下拉（手动/导航）、连接状态
- **手动模式**：摇杆（斜推=边走边转，转弯自动降速）、急停、零速测试、状态卡
- **导航模式**：
  - 地图（最近一次导航的 PGM）以屏幕适配显示，双指缩放/旋转、单指平移、＋/－按钮、比例尺
  - 轻点加航点 → 自动请求**路线预览**（浅蓝虚线，车不动）
  - 「开始导航」→ 下发全部航点（蓝色实线=全局路径，绿色虚线=局部避障）
  - 「停止导航」「清除航点」「📍 重新定位（一键播种）」
  - 顶部进度：`导航已接受：共 N 个航点` → `导航中：正在前往 N/M 号航点` → `导航完成`
- 车体按真实尺寸绘制：**0.52 m × 0.36 m**，四角圆角 0.06 m，车头红色标志
- 激光点黄色、预览浅蓝虚线、全局路径蓝色实线、局部轨迹绿色虚线

## 5. 底盘参数（换新主控 + 100:1 电机后）

| 项 | 值 | 说明 |
| --- | --- | --- |
| `track_m` | **0.317** | 卷尺 + 旋转标定一致 |
| `circumference_m` | **0.314159** | 100 mm 轮 |
| `counts_per_rev` | **6400** | 16PPR × 4 × 100:1 |
| `wheel_speed_scale` | **1.0** | 固件已重标 |
| `manual_max_speed_mps` | **0.30** | 手动/遥控单轮包络 |
| 固件 pivot 占空比 | 按新电机重标 | 旧值 0.56 会让原地转向快 8 倍 |

## 6. 已知限制 / 待办

1. **域名未备案**：`https://jccontrol.xyz` 不可用；备案下来后启用 443 + 证书。
2. **EMQX 仍是匿名**（船的控制台依赖匿名）；dashboard 18083 对公网开放，建议安全组限源。
3. **地图是“最近一次导航用的地图”**，来自 `~/.config/fwr/last_navigation_map`；
   换地图后重启 `fwr-mqtt-bridge` 即可重新上传。
4. **航点编号是近似值**：探索器未发布当前段号，页面用“离车最近的航点”推算。
5. **视频未接入**：SRS 容器已存在（1935/8081），后续接 WebRTC。
6. 电量：JSY-MK-1031 已接入（见下节），服务 `fwr-battery.service` 开机自启。
7. 建图：`fwr-map.service` 建图栈（见 6.5 节），手机端可摇杆建图、保存/重建地图。

## 6.5 建图模式（2026-11-16 增补）

模式下拉新增「建图模式」，页面结构：上方 SLAM 地图+雷达点，下方摇杆+按钮。

| 按钮 | 作用 |
| --- | --- |
| 授权 | 打开手动门（`/base_bridge` manual_enabled）|
| 保存地图 | 弹框输入名字 → `/mapping/save_slam`，存 pgm+yaml+posegraph，并设为下次导航地图 |
| 重新建图 | `systemctl restart fwr-map`（清空当前地图）|
| 急停 | 零速 + 摇杆复位 |

车端切换：`fwr-map.service`（建图栈）↔ `fwr-nav.service`（导航栈），约 40~120 秒。

- SLAM 地图走独立话题 **`/slam_map`**（slam_toolbox 重映射），桥接只在
  `fwr-map` 运行时上传 `live_map`，切换后 25 秒宽限期内不上传，
  避免混入其它节点在 `/map` 上重发的导航地图。
- 建图模式没有 AMCL，车体位姿来自 TF `map→base_link`。
- 激光点屏蔽车尾 ±50°（`rear_mask_deg`），与 RViz `/scan_mapping` 一致。
- 视角切换（🧭）：固定地图 / 车头朝上（地图随车旋转，车固定在屏幕中心）。

保存地图实体：`maps/<名字>_<时间戳>.pgm/.yaml/.posegraph/.data`；
保存后自动写入 `~/.config/fwr/last_navigation_map`。

## 7. 电池计量（JSY-MK-1031，2026-11-15 接入）

### 接线（串联在电池正极）

```
电池 P+ ──────────► 模块 L（输入粗焊盘）
模块输出粗焊盘 ───► 驱动板 P+
电池 P− ──┬───────► 驱动板 P−
          └───────► 模块 N 引脚（细针，电压采样，不走电流）
模块 VCC/GND ─────► USB 转接板 5V/GND（隔离，不与电池共地）
模块 TXD/RXD ─────► USB 转接板 RXD/TXD（交叉）
```

模块 16 A / 1.2 倍过载；电池总线 15 A 自恢复保险丝，保险丝先动作。

### 设备名（两个 CH340 撞 ID，用 udev 按 USB 端口区分）

`/etc/udev/rules.d/99-fwr-serial.rules`：

```
SUBSYSTEM=="tty", KERNELS=="3-1.1", SYMLINK+="jsy_meter"
SUBSYSTEM=="tty", KERNELS=="3-1.3", SYMLINK+="ultrasonic"
```

### 关键坑：直流模式要写 **2**，不是手册写的 1

- 写 `0x0005 = 1`：模块回 OK，但型号寄存器仍是 `0x0105`（交流），电压恒读 0
- 写 `0x0005 = 2`：型号变成 `0x0205`（直流），电压立刻正确（实测 22.18 V）
- 切换后模块会短暂重启，**要等约 0.7 秒**再读

### 当前读数

```
电压 22.18 V   电流 0.47 A   功率 10.4 W   电量约 39%
（6S 锂电，22.18/6 = 3.70 V/节）
```

服务：`fwr-battery.service`（`ros2 run fwr_bringup battery_modbus -p device:=/dev/jsy_meter`），
发布 `/base/battery`，由 `mqtt_bridge` 汇总进手机端状态。
