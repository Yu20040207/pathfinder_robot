---
name: fwr-workflow
description: 四轮机器人二代项目的开工、验证与收尾流程。用于本项目任何开发、调试、构建、测试、仿真或实机接入任务；强调先读交接记录避免重复排查，按改动类型跑最小验证，未经授权不驱动实机。
---

# 四轮机器人项目工作流

## 何时使用

在本仓库（`Four-wheel-robot`）里做任何需要改代码、跑测试、启动 ROS、接硬件、排查故障或交接会话的任务时，先按本流程走。它不能替代 `AGENTS.md`，两者冲突时以用户当前指令和 `AGENTS.md` 的安全红线为准。

## 1. 开工：先接上上次的进度（约 2 分钟）

```bash
cd ~/Company_Project/Four-wheel-robot

# 1) 现状：不要假设上次已经提交
git status --short | head -40
git log --oneline -5

# 2) 上次会话的结论、证据和运行状态（只读最后 2–3 条）
tail -120 docs/AGENT_LOG.md

# 3) 若属于某个计划，读进度账本
ls -t .superpowers/sdd/*/progress.md | head -5
# 选最新的读，例如：
# tail -40 .superpowers/sdd/2026-10-21-directional-collision-monitor/progress.md
```

判断“这件事是否已经做过”的顺序：

1. `progress.md` 里标 `complete` 的任务和 commit 范围；
2. `AGENT_LOG.md` 最后几条的“已验证/未验证”；
3. `git log -S '<关键词>'` / `grep -rn '<关键词>' docs ros2_ws/src firmware`；
4. 都没有，再开始新的排查。

## 2. 定位：先搜后读，别通读

```bash
# 按文件名找
find ros2_ws/src/fwr_bringup -maxdepth 2 -name '*.py' | head

# 按内容找（优先用项目的 grep 工具；命令行等价）
grep -rn "class FrontierExplorer\|def parse_state_line" ros2_ws/src/fwr_bringup | head -20

# 只读需要的行
sed -n '1,120p' ros2_ws/src/fwr_bringup/fwr_bringup/base_bridge.py
```

不要读：`build/`、`install/`、`log/`、`.venv/`、`ascam_ros2_ws/third_party/`、`data/`、`models/`、`*.log`。
大输出先落盘再 grep：`cmd > /tmp/out.txt 2>&1; grep -n ... /tmp/out.txt | head`。

## 3. 改动的验证矩阵

先跑最小集，通过后再扩大；**全量只在交接前跑一次**。

| 改动 | 命令 |
| --- | --- |
| 纯 Python 核心/协议/几何 | `PYTHONPATH=ros2_ws/src/fwr_bringup .venv/bin/python -m pytest -q ros2_ws/src/fwr_bringup/test/test_frontier_core.py`（换成对应文件） |
| 多个 Python 节点 | 先跑相关 test 文件，再 `source /opt/ros/humble/setup.bash && .venv/bin/python -m pytest -q tests ros2_ws/src/fwr_bringup/test` |
| ROS 包/接口/launch | `source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build --packages-select fwr_interfaces fwr_bringup --symlink-install` |
| launch 语法 | `.venv/bin/python -m py_compile ros2_ws/src/fwr_bringup/launch/real_navigation.launch.py` |
| ESP32-S3 固件 | 见 `firmware/esp32s3_base_control/README.md` 的主机测试与 `idf.py build` |
| 仿真 | 隔离 `ROS_DOMAIN_ID`，日志放 `/tmp/fwr_*.log`；不要和实机抢 `/scan`、`/cmd_vel` |

记录证据时写全：命令、结果（`N passed` / build 成功）、日志路径。**离线测试通过 ≠ 实机验证通过。**

## 4. 实机安全清单

在发任何运动命令前逐项确认：

- [ ] 用户在当前会话明确同意做这项实机动作；
- [ ] 车轮悬空或场地无人，电源可立即断开；
- [ ] 烧录/接线时电机 12 V 已断开；
- [ ] `base_bridge` 的 `motion_enabled` / `manual_enabled` 仍为 false，除非本次就是要测试运动；
- [ ] 没有第二个 `sllidar_node` / `dm_imu_node` 在重复发布；
- [ ] 雷达用 `/dev/serial/by-id/...`，不是写死的 `/dev/ttyUSB0`；
- [ ] 测试结束发停止命令，确认 `active=0`、PWM=0。

不要用 `ros2 topic pub /cmd_vel` 直接驱动实机；实机运动必须经过项目已有的 gate / bridge / 碰撞监控链路。

## 5. 调试循环（避免重复试错）

1. **观察**：只取最小证据（`ros2 topic hz --once`、`ros2 topic echo --once`、日志最后 30 行）。
2. **假设**：写下“如果 X 则 Y”；一次只验一个变量。
3. **最小实验**：优先纯函数测试或隔离仿真；不要一上来重启整棵 Nav2。
4. **记录**：成功/失败都写进 `progress.md` 或 `AGENT_LOG.md`，附命令与日志路径。
5. **收敛**：连续两次同类失败就停下来，向用户报告事实和选项，不要继续加电/加参数碰运气。

## 6. 省 token 的具体做法

- 一次消息里并行发互不依赖的 `read`/`grep`/`bash`；不要串行等。
- 需要大范围探索时用 subagent，提示词写清“只回传 ≤20 行结论 + 文件行号”。
- 不要重复读刚写过的文件；以工具返回为准。
- 不要反复重跑同一测试；代码没变就不跑。
- 长会话在阶段边界 `/compact`；新任务开新会话，靠 `AGENT_LOG.md` 交接，而不是靠拖长上下文。
- 不要会话中途改 `AGENTS.md`；收尾时一次性改。

## 7. 收尾：留下可接手的现场

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

追加到 `docs/AGENT_LOG.md` 末尾，不要删除历史。更新对应 `progress.md`。只有长期有效的新事实才回写 `AGENTS.md`。
