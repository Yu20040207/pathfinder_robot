# ROS MCP 只读接入记录

> 2026-10-24 安装。目标：让 DSH 能直接读 ROS 话题/节点/参数，减少“人肉复制 `ros2 topic echo`”的 token 和时间；**不**给模型发布速度、调服务、改参数的入口。
> 当前状态：**已启用并通过只读端到端验证。** `rosbridge 2.0.8` 已安装并在 9090 运行；DSH 已热加载 `mcp__ros__*` 的 11 个只读工具。

## 1. 已安装什么

| 组件 | 位置 | 说明 |
| --- | --- | --- |
| `ros2-engineering-skills` | `~/.agents/skills/ros2-engineering-skills` | 第三方 Agent Skill，Apache-2.0，约 2.8 MB；DSH 已识别 |
| `ros-mcp` 3.1.2 | `~/.local/share/fwr-ros-mcp/venv` | 独立 venv，避免污染项目 `.venv` 和系统 Python |
| 只读包装 | `~/.local/bin/ros-mcp-readonly` | 调 venv 里的 `ros_mcp_readonly.py`；默认只暴露 11 个只读工具 |
| 包装源码 | `~/.local/share/fwr-ros-mcp/ros_mcp_readonly.py` | 白名单在启动前移除写工具，失败即退出 |
| 探测脚本 | `~/.local/share/fwr-ros-mcp/probe.py` | 只列工具，不连机器人、不发布任何东西 |
| DSH 配置行 | `~/.dsh/profiles/web/cordis.patch.yml` | `mcp-ros`，`disabled: false`（已启用） |
| rosbridge | 系统包 `ros-humble-rosbridge-server 2.0.8` | 监听 9090；由用户终端启动，关掉终端就会断开 |

安装来源与校验：

- skill：`https://github.com/dbwls99706/ros2-engineering-skills`，commit `f0de84dfedd67b067ccc6d2851e681529606930c`（2026-10-20）；下载 tarball SHA256 `857a48f48aedeb43ac772ca520e9907e9d989d728d2e7acec131ccb5af8621ff`。
- `ros-mcp`：PyPI `ros-mcp==3.1.2`（Apache-2.0）。
- DSH profile 备份：`~/.dsh/profiles/web/cordis.patch.yml.bak-*`。

## 2. 为什么保持只读

1. **ros-mcp 原生有 31 个工具**，包含 `publish_once`、`publish_for_durations`、`call_service`、`set_parameter`、`send_action_goal` 等写操作；当前版本没有内置只读/权限模型。
2. **只读包装是最后一道门**：在 MCP 服务启动前按白名单移除所有写工具，并检查没有遗漏；即使上游新增工具，默认也不会暴露。
3. **上下文成本**：11 个只读工具约 4.6 KB Schema；如果把 31 个都挂上，既增加 token，也增加误操作面。

## 3. 只读包装的三种模式

通过环境变量 `FWR_ROS_MCP_MODE` 控制：

| 模式 | 工具数 | 内容 |
| --- | --- | --- |
| `min`（默认） | 11 | connect、topics、message details、subscribe、nodes、services、parameters |
| `readonly` | 21 | 在 min 基础上加 actions、参数详情、服务详情等只读项 |
| `full` | 31 | 保留全部原生工具；**必须同时设 `FWR_ROS_MCP_ALLOW_WRITE=1`**，否则拒绝启动 |

包装在启动前移除所有不在白名单里的工具，并再次检查；发现意外工具就退出。`view_saved_image` 不在默认白名单里（它能读本地图片路径，且当前主模型未必能看图）。

验证：

```bash
cd ~/.local/share/fwr-ros-mcp
./venv/bin/python probe.py                 # mode=min tool_count=11
FWR_ROS_MCP_MODE=readonly ./venv/bin/python probe.py   # tool_count=21
FWR_ROS_MCP_MODE=full ./venv/bin/python probe.py       # 拒绝启动，exit 2
```

## 4. rosbridge（已安装）

2026-10-24 已由用户安装 `ros-humble-rosbridge-server 2.0.8`，并在终端启动：

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

当前监听 `0.0.0.0:9090`，即所有网卡都可访问。**建议本机使用时重启为只监听回环：**

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1
```

安全建议：

- rosbridge 默认没有鉴权，**不要暴露到公网**；只在本机/可信局域网使用。
- 如果 DSH 和 ROS 同机，`address:=127.0.0.1` 足够。
- 先不要在实机导航栈运行时开放写工具；只读包装就是为此。

## 5. DSH 启用状态（已启用）

- `~/.dsh/profiles/web/cordis.patch.yml` 里的 `mcp-ros` 已是 `disabled: false`。
- DSH 的 HMR 会监听 profile 配置；本次修改后**无需重启**，下一次请求就出现了 `mcp__ros__*`。如果以后没出现，再重启 DSH。
- 工具列表只有 11 个只读工具：`connect_to_robot`、`detect_ros_version`、`get_topics`、`get_topic_type`、`get_topic_details`、`get_message_details`、`subscribe_once`、`subscribe_for_duration`、`get_nodes`、`get_services`、`get_parameters`。
- 没有 `publish_once`、`publish_for_durations`、`call_service`、`set_parameter`、`send_action_goal` 等写工具。

### 端到端验证（2026-10-24 11:00）

```text
mcp__ros__connect_to_robot(127.0.0.1, 9090) → Fully_accessible
mcp__ros__get_nodes()                       → 35 个节点
mcp__ros__get_topic_type("/scan")           → sensor_msgs/msg/LaserScan
mcp__ros__get_parameters("/base_bridge")    → 5 个参数
subscribe_once("/scan")                     → frame_id=laser_front, 720 点
```

只读包装独立探测也通过：`min=11`、`readonly=21`、`full` 无 `ALLOW_WRITE=1` 时拒绝启动。

如果以后想临时关闭：把 `mcp-ros` 的 `disabled` 改回 `true`；HMR 会卸载工具，**不用重启**，同时省下每轮约 4.6 KB 的 Schema。

## 6. 常用只读调用示例

```text
connect_to_robot(ip="127.0.0.1", port=9090)
get_topics()
get_topic_type(topic="/scan")
get_topic_details(topic="/scan")
subscribe_once(topic="/scan", msg_type="sensor_msgs/msg/LaserScan")
subscribe_for_duration(topic="/imu/data_raw", msg_type="sensor_msgs/msg/Imu", duration=1.0)
get_nodes()
get_parameters(node_name="/base_bridge")
```

如果工具名/参数与版本不一致，以 `probe.py` 列出的 Schema 为准。

## 7. 回滚

```bash
# 1) 禁用 DSH 行：把 disabled 改回 true，或恢复备份
ls ~/.dsh/profiles/web/cordis.patch.yml.bak-*
# cp ~/.dsh/profiles/web/cordis.patch.yml.bak-YYYYmmdd-HHMMSS ~/.dsh/profiles/web/cordis.patch.yml

# 2) 删除客户端和包装
rm -rf ~/.local/share/fwr-ros-mcp ~/.local/bin/ros-mcp-readonly

# 3) 删除 skill
rm -rf ~/.agents/skills/ros2-engineering-skills
```

项目仓库内没有因这次安装改动任何源码；只有本文和 `AGENT_LOG.md` 的记录。
