# 房间标号（语义标签）

> 2026-10-24 新增。目的：给地图上的房间一个稳定的编号和名字，并在 RViz 中显示文字标记；**不改 PGM 图片**，避免标号和栅格地图绑死。

## 为什么不用改地图图片

`map_server` 加载的 `.pgm/.yaml` 只是占据栅格，没有“房间”这个概念。直接在 PGM 上画数字，Nav2 读不到，换分辨率或重新建图还会丢。正确做法是单独存一份语义 YAML，地图仍保持纯栅格。

## 文件

| 文件 | 作用 |
| --- | --- |
| `ros2_ws/src/fwr_bringup/maps/rooms.yaml` | 房间编号、名称、地图坐标、朝向 |
| `scripts/room_labels.py` | 订阅 RViz `/clicked_point`，提示输入编号/名称，写 YAML 并发布 `/room_labels` |
| `/room_labels` | `visualization_msgs/MarkerArray`，RViz 显示文字和圆点 |
| `real_mapping.rviz` / `real_localization.rviz` | 已加入“房间标号”MarkerArray 显示 |

## YAML 格式

```yaml
frame_id: map
rooms:
- id: 1
  name: 客厅
  x: 1.25
  y: -0.4
  yaw: 0.0
- id: 2
  name: 厨房
  x: -0.5
  y: 1.25
  yaw: 1.5708
```

- `id`：稳定整数，建议从 1 开始；同一个房间不要换号。
- `name`：中文/英文都行，仅用于显示和后续语义导航。
- `x/y`：房间中心在 `map` 坐标系下的坐标，单位米。
- `yaw`：进入/面向房间的朝向，单位弧度；脚本里输入的是角度，自动转换。

## 使用步骤

1. 地图建好并保存后，保持 RViz 显示 `/map`，Fixed Frame 设为 `map`。
2. 在一个已 source ROS 的终端运行：

```bash
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
python3 /home/manbaout/Company_Project/Four-wheel-robot/scripts/room_labels.py
```

3. 在 RViz 工具栏选择 **Publish Point**，点击房间中心。
4. 终端会提示：

```text
点击坐标：x=1.250  y=-0.400  (frame=map)
房间编号（整数，直接回车跳过）: 1
房间名称（回车=房间1）: 客厅
朝向 yaw（度，可选，0=地图 +x；回车=0）: 0
```

5. 每输入一个房间，YAML 立即保存，RViz 里出现“1 客厅”的文字和蓝色圆点。
6. 重复点击/输入，直到所有房间标完；Ctrl-C 退出。

如果已有 YAML，脚本启动时会加载并重新发布标记；再次点同一编号会覆盖旧标签。

## 在 RViz 里查看

`real_mapping.rviz` 和 `real_localization.rviz` 已包含名为“房间标号”的 MarkerArray，话题 `/room_labels`。如果没有显示：

- 确认脚本仍在运行（它每秒重发一次标记，方便后打开的 RViz 接收）；
- Fixed Frame 必须是 `map`；
- MarkerArray 的 Durability 选 `Transient Local`，Reliability 选 `Reliable`；
- 检查 `/room_labels` 有 publisher：`ros2 topic info /room_labels`。

## 和导航的关系

目前房间标签是**语义层**，还没有接入导航节点。现阶段可以：

- 用标签 YAML 记录房间中心，人工在 RViz 单点选择该坐标；
- 后续写一个 `room_nav` 节点，读取 `rooms.yaml`，按 `id/name` 调 `/explore/start_fixed_goal` 或直接发 `NavigateToPose`；
- 不要假设 `id` 会自动变成 Nav2 的 waypoint；Nav2 的 waypoint 文件是另一套格式。

## 注意

- 标签坐标属于**建图时的 map 坐标系**。重新建图或地图原点变化后，旧标签会错位，需要重新标。
- 标号是给人看和给上层逻辑用的，不改变碰撞、代价地图或安全门。
- 如果房间很多，建议先定编号规则（例如 1xx 客厅、2xx 厨房），不要用点击顺序临时编号。
