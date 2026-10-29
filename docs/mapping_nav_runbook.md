# 逐步操作手册：换橡胶轮 → 标定 → 建图 → 导航

> 每一步都给到「命令 / 期望结果 / 不通过怎么办」。
> 配套背景与参数依据见 `docs/mapping_nav_workflow_2026-10-30.md`。

---

## 步骤 0：每次开新终端先执行的环境变量（后面所有命令都依赖）

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.config/fwr/fastdds_udp_only.xml
cd /home/manbaout/Company_Project/Four-wheel-robot
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

**判据**：`ros2 pkg list | grep fwr_bringup` 有输出（说明工作区已 source）。

**不通过**：说明没 `colcon build` 或 source 路径不对 → 先 `cd ros2_ws && colcon build --symlink-install`。

---

## 步骤 1：确认硬件都在线（30 秒）

```bash
ls /dev/serial/by-id/ | sed 's/usb-//'
```

**必须看到 3 个**：

| 设备名开头 | 是什么 |
|---|---|
| `DM-Tech_DM-IMU-L1_...` | IMU |
| `Espressif_USB_JTAG_...` | **ESP32 底盘（最关键）** |
| `Silicon_Labs_CP2102N_...` | 雷达 |

**不通过**：

```bash
lsusb | grep -i espressif     # 有输出说明系统认到 ESP32
```
- 没有输出 → 换数据线 / 换 USB 口（别走扩展坞）/ 底盘断电 5 秒再上电
- 有 3 个设备但节点起不来 → 检查是否残留旧进程：
```bash
ps -e -o cmd= | grep -E "sllidar|dm_imu|fwr_bringup|ros2 launch" | grep -v grep
```
  有残留就先杀掉再继续（**注意：`sllidar` 的路径是 `install/sllidar_ros2/`，
  用 `fwr_` 匹配杀不到**）。

---

## 步骤 2：换橡胶轮 + 物理检查

1. 换四个橡胶驱动轮（**同直径、同宽度**）。
2. 擦净轮面、擦净地面（细沙会让打滑翻倍）。
3. **手拨前后两个万向轮**：应能自由转向，卡死会增加阻力。
4. 检查线缆/护板没有拖地。

**判据**：手推机器人（断电）应能顺畅直线滑动，无明显阻滞感。

---

## 步骤 3：起底盘栈（标定用的最小系统）

```bash
ros2 launch fwr_bringup real_mapping.launch.py start_slam:=false
```

这是「只开底盘+雷达+IMU+EKF，不开建图」的最小栈，标定期间**不要**开导航。

**判据**（另开终端，先做步骤 0）：

```bash
ros2 topic hz /scan        # 约 10 Hz
ros2 topic hz /imu/data    # 约 100 Hz
ros2 topic hz /wheel/odom  # 约 10 Hz
```

**不通过**：
- 缺 `/wheel/odom` → ESP32 没连上（回步骤 1）
- 缺 `/imu/data` → IMU 串口被占用，或有两个 `dm_imu` 进程（杀掉多余的那个）
- 缺 `/scan` → 雷达串口被占用（同上，查重复进程）

开运动门（后面标定要用）：

```bash
ros2 param set /base_bridge motion_enabled true
ros2 param set /base_bridge manual_enabled true
ros2 topic echo /base/ready --once     # 应为 data: true
```

---

## 步骤 4：标定轮子周长（换胎后必做，最重要）

### 4.0 先搞清楚：标的是「有效周长」，不是拿尺子量轮胎

| 概念 | 怎么得到 | 用途 |
|---|---|---|
| 几何周长 | 卡尺量轮胎直径 D，C = π·D | 只能当**初值**参考 |
| **有效周长（要标定的）** | **开一段量好的直线，用「实际距离 / 里程计读数」反推** | 直接决定里程计和地图尺度 |

为什么要用「跑一段」而不是量轮胎：有效周长包含**轮胎受压变形、打滑、胎面花纹**的影响，
拿尺子量出来的只是钢圈+橡胶的外径，和实际滚动一圈走过的距离能差 1~3%。

**和之前测 1.5 m 是同一个方法**，只是建议把基准拉长到 **2~3 m**：
卷尺误差 5 mm 在 1.5 m 上是 0.33%，在 3 m 上只有 0.17%。

### 4.0b （可选）滚一圈直接量，作为初值

1. 在轮胎侧面用记号笔点一个标记，对应地面也点一个。
2. 断开电机电源，用手推车让轮子**正好滚一整圈**（标记回到地面）。
3. 量两个地面点之间的距离 = 有效周长的粗略值（±1%）。

### 4.1 准备

- 在**地面贴两条胶带**，间距用卷尺量准：建议 **3.000 m**（越长越准）。
- 机器人车头对准第一条胶带。

### 4.2 采集

```bash
.venv/bin/python tools/odom_calibrate.py
```

流程：
1. 程序启动后**按回车**（开始记录）
2. 用另一个终端（步骤 0 后）起键盘：
```bash
ros2 run fwr_bringup keyboard_teleop --ros-args -p forward_speed_mps:=0.12 -p turn_rate_rad_s:=0.4
```
   按住 `w` 慢速直行到第二条胶带，**停在胶带正中**，松开。
3. 回到 odom 工具，**按回车**（结束记录）
4. 按提示输入**实际距离 3.0**

### 4.3 计算与修改

工具会打印类似：

```
odom 读到的距离 = 3.12 m（实际 3.00 m）
建议圆周 = 0.2012 × (3.00 / 3.12) = 0.1935
```

修改文件：`ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge_core.py`

```python
def __init__(self, track_m: float = .268, circumference_m: float = .2012, ...):
                                                    ↑ 改成新值
```

> ⚠️ `/wheel/odom` 是 **base_bridge** 发的，不是 `wheel_odometry_node.py`，改错文件不生效。

重新编译并重启栈：

```bash
cd ros2_ws && colcon build --packages-select fwr_bringup --symlink-install
```

### 4.4 复验

**重新做一次 4.2**（回到起点重来）。

**判据**：里程计读数与实测距离误差 **<1%**（3 m 走 2.97~3.03）。

**不通过**：
- 误差 5~20% → 打滑仍然严重 → 回步骤 2 检查轮面/万向轮/地面
- 误差 >30% → 检查 `counts_per_rev`（编码器每圈计数）是否写错

**做 3 次取平均**，每次从起点重来。

---

## 步骤 5：标定轮距（track）

### 5.1 准备

- 原地转圈需要的空间：**四周 ≥0.8 m**（车头扫掠半径约 0.37 m）。
- 用雷达确认：把车停好后看空间是否足够（我可以帮你测，或看 RViz）。

### 5.2 执行

```bash
.venv/bin/python tools/track_calibrate.py 90 0.35
```

机器人会自动正转 90°、反转 90°，约 20 秒。

**判据输出**：

```
正向比值 1.00xx   反向比值 1.00xx   差异 <1%
建议轮距 = 0.268 × 1.00xx = 0.2xxx m
```

合格：**比值 0.998~1.002**。

### 5.3 修改与复验

改 `base_bridge_core.py` 的 `track_m=` → `colcon build` → 重启栈 → 再跑一次
`tools/track_calibrate.py 90 0.35`，比值应落在 0.998~1.002。

**不通过**：
- 正反差异 >2% → 某侧打滑更严重（检查轮胎/轮径一致性）
- 比值始终偏离 → 检查 `track_m` 单位是否写错（米，不是毫米）

---

## 步骤 6：打滑率复核（换轮效果的直接证据）

```bash
# 键盘 0.12 m/s 直行 3 秒，量车体实际位移
```

**判据**：

| 有效率 | 结论 |
|---|---|
| >90% | ✅ 换轮成功 |
| 70~90% | 尚可，可建图但尽量慢行 |
| **<70%** | ❌ 别急着建图（今天就是 63%），回步骤 2 检查 |

---

## 步骤 7：IMU 体检（2 分钟）

### 7.1 静止漂移

机器**完全静止** 60 秒，看航向变化：

```bash
# 我可以直接跑给你看：静止 60s 后比较首尾航向
```
**判据**：漂移 <0.5°/min。

### 7.2 转一圈一致性

```bash
.venv/bin/python tools/track_calibrate.py 180 0.35
```
**判据**：编码器与陀螺仪积分的比值靠近 1.0（这个值同时验证了步骤 5）。

---

## 步骤 8：建图（slam_toolbox）

### 8.1 停止标定栈，起建图栈

```bash
# 先 Ctrl-C 掉步骤 3 的栈，然后：
ros2 launch fwr_bringup real_mapping.launch.py
```

**判据**：

```bash
ros2 node list | grep slam            # 有 slam_toolbox
ros2 topic info /map | grep Publisher # 必须是 1
```

### 8.2 开 RViz 看图

```bash
rviz2 -d ros2_ws/src/fwr_bringup/config/real_mapping.rviz
```

### 8.3 开运动门 + 键盘

```bash
bash tools/kb.sh      # 自动开门并起键盘
```

### 8.4 起步（关键）

- **先在开阔处直行 1 米**，看着 RViz 里地图开始生长
  （slam_toolbox 只按**平移**加节点，原地转不建图）
- 确认地图里出现墙线后再开始正式扫

### 8.5 扫图规矩

| 要点 | 数值/做法 |
|---|---|
| 直行速度 | 0.10~0.12 m/s，**点按 1~2 秒** |
| 转弯 | 先停 1 秒 → 0.35 rad/s → 转完停 1 秒 |
| 离墙 | 沿走廊中线，约 0.5 m |
| 门洞 | 进/出各走一遍；经过时**稍微转头看门内**（给匹配纵向锚点） |
| 走廊 | 慢行，**不要原地转圈** |
| **收尾** | **必须开回起点闭环**，再往前 1~2 m |

### 8.6 实时监控（另开终端）

```bash
tail -f /tmp/fwr_health.log     # 我帮你起的漂移监视
```
**判据**：`map→odom 修正 旋转 <5°`。
**>10° 立即停车**，说明地图已经开始歪。

### 8.7 存图（停车后！）

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/fwr_maps/loop_run'}"
ros2 run nav2_map_server map_saver_cli -f $HOME/fwr_maps/loop_run
```

> ⚠️ **不要边跑边存**：高频 `serialize_map` 会让 slam_toolbox 崩溃
> （`Mapper FATAL ERROR`，见 issue #642）。

### 8.8 安装到工作区

```bash
cp ~/fwr_maps/loop_run.pgm ros2_ws/src/fwr_bringup/maps/new_map.pgm
sed 's/loop_run.pgm/new_map.pgm/' ~/fwr_maps/loop_run.yaml \
  > ros2_ws/src/fwr_bringup/maps/new_map.yaml
cd ros2_ws && colcon build --packages-select fwr_bringup --symlink-install
```

**不建编译会报 `bad file`**（map_server 找不到新图）。

---

## 步骤 9：定位（AMCL + 播种）

### 9.1 起导航栈

```bash
ros2 launch fwr_bringup real_navigation.launch.py \
  use_saved_map:=true localizer:=amcl \
  map:=$PWD/ros2_ws/install/fwr_bringup/share/fwr_bringup/maps/new_map.yaml
```

**判据**：

```bash
ros2 topic info /map | grep Publisher        # 1
ros2 lifecycle get /amcl                     # active
ros2 lifecycle get /planner_server           # active
```

### 9.2 播种（两种方式任选）

**方式 A（推荐，自动匹配）**：

```bash
.venv/bin/python tools/seed_amcl.py \
  ros2_ws/src/fwr_bringup/maps/new_map.yaml --publish
```
**判据**：`score < 0.10 m`，然后 `map→base` TF 出现。

**方式 B（手动）**：RViz 工具栏 `2D Pose Estimate` → 在地图上点机器人真实位置 → 拖动定朝向。

### 9.3 确认定位正确

看 RViz：**蓝色雷达点应贴在灰色墙线上**。错位就重新播种。

---

## 步骤 10：导航验证

### 10.1 近目标（2~3 m）

RViz 里单击地图空地（离墙 >0.5 m）→ 面板点「开始导航」。

**判据**：车开始走，`ros2 topic hz /cmd_vel` 有输出。

### 10.2 远目标

近目标成功后，再点 5~8 m 的目标。

### 10.3 出问题怎么看

| 现象 | 原因 | 处理 |
|---|---|---|
| `no valid path found` | 目标在墙里/被隔断，或起点贴墙 | 换目标点；或先把车开到开阔处 |
| `detected collision ahead` | 前方/侧向空间小于膨胀半径 | 观察 RViz，把车挪到开阔处再点 |
| `spin failed` | 周围太空旷或太窄，旋转无进展 | 同上 |
| 车不动、日志有 `not-armed` | 授权丢了 | 面板「清空状态/复位故障」后重新「开始导航」 |
| `fault 1 失速` | 打滑/被卡 | `bash tools/fault_reset.sh` 然后重新起键盘 |

---

## 步骤 11：收尾（每次实验都做）

```bash
# 1) 停车：Ctrl-C 掉键盘节点
# 2) 关运动门
ros2 param set /base_bridge motion_enabled false
ros2 param set /base_bridge manual_enabled false
# 3) 保存本次地图与位姿图
bash tools/map_save.sh
```

---

## 附：常用工具速查

| 工具 | 用途 |
|---|---|
| `bash tools/kb.sh` | 开运动门 + 起键盘（建图用） |
| `bash tools/fault_reset.sh` | 失速/堵转后清故障 + 重新开门 |
| `bash tools/map_pause.sh` | 冻结/解冻建图（看到重影立刻冻结） |
| `bash tools/map_rollback.sh 2` | 回退到几秒前的位姿图快照（slam_toolbox） |
| `python tools/odom_calibrate.py` | 轮子周长标定 |
| `python tools/track_calibrate.py 90 0.35` | 轮距标定 |
| `python tools/seed_amcl.py <map.yaml> --publish` | 给 AMCL 播种 |
| `python tools/map_health.py` | 监视 map→odom 漂移 |
