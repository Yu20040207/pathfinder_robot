# 本项目已安装的辅助工具（2026-11-04）

> 全部**不需要 sudo**：命令行工具在 `~/.local/bin`（已在 PATH 里），
> Python 工具在项目 `.venv`。
> 需要 sudo 才能装的 ROS 图形工具见第 4 节（本机 sudo 需要密码，未安装）。

---

## 1. 命令行工具（`~/.local/bin`）

| 工具 | 版本 | 在本项目里干什么 | 典型用法 |
| --- | --- | --- | --- |
| `fzf` | 0.74.5 | 模糊搜索：找日志、找文件、翻命令历史 | `fzf` / `vim $(fd launch.py \| fzf)` |
| `fd` | 10.5.0 | 比 `find` 快得多的文件查找（自动忽略 build/install） | `fd -e launch.py real_` |
| `bat` | 0.26.1 | `cat` + 语法高亮 + 行号，看日志/配置更清楚 | `bat -n docs/AGENT_LOG.md` |
| `hyperfine` | 2.0.0 | 命令基准测试（比手掐秒表准） | 见下面"测耗时" |
| `btop` | 1.4.8 | 比 `top` 好用的资源监视（**排查 rviz2/节点吃满 CPU** 时很有用） | `btop` |

### 结合本项目痛点的几个用法

```bash
# ① 快速定位某个参数被哪个文件设置（比 grep 全仓快）
fd -e yaml -e launch.py | fzf --preview 'bat --color=always -n {} | grep -n inflation'

# ② 看大日志只看关键段落
bat -n /tmp/fwr_runs/xxx.log | grep -nE "inflation|front_stop" | head

# ③ 测测试套件耗时（改完什么可以量一下有没有变慢）
hyperfine --warmup 1 \
  'PYTHONPATH=ros2_ws/src/fwr_bringup .venv/bin/python -m pytest ros2_ws/src/fwr_bringup/test -q'

# ④ 查是谁把 CPU 吃满了（我们遇到过 rviz2 33% + 多个节点抢 CPU → TF 滞后）
btop
```

---

## 2. Python 工具（`.venv`，用 `.venv/bin/<name>` 调用）

| 工具 | 版本 | 用途 |
| --- | --- | --- |
| `ruff` | 0.17.3 | **静态检查：抓未定义变量、未使用导入** |
| `pre-commit` | 4.6.2 | git 提交前自动检查（**尚未配置 hook，需你同意再加**） |
| `pytest-xdist` | — | 并行跑测试（`-n auto`）|
| `pytest-html` | — | 生成 HTML 测试报告 |

### ruff（已配好 `ruff.toml`，只查真 bug，不查风格）

```bash
# 查整个 bringup 包
.venv/bin/ruff check ros2_ws/src/fwr_bringup/fwr_bringup/

# 只修"能安全修"的（未使用的导入）
.venv/bin/ruff check ros2_ws/src/fwr_bringup/fwr_bringup/ --fix
```

配置见 [`ruff.toml`](../../ruff.toml)：只选 `F`（未定义变量/未使用导入）+ `E9`（语法错误），
**故意不选风格规则**，避免对已稳定运行的项目产生大批无意义改动。

### pytest-xdist / pytest-html

```bash
# 并行（测试多了以后有收益；现在 255 个只要 1.4 s，串行即可）
PYTHONPATH=ros2_ws/src/fwr_bringup .venv/bin/python -m pytest ros2_ws/src/fwr_bringup/test -q -n auto

# 生成可点开的报告
PYTHONPATH=ros2_ws/src/fwr_bringup .venv/bin/python -m pytest ros2_ws/src/fwr_bringup/test \
  --html=/tmp/fwr_test_report.html --self-contained-html
```

> ⚠️ 跑测试必须**先 source ROS**，否则会有一堆 import 错误：
> ```bash
> source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
> PYTHONPATH="ros2_ws/src/fwr_bringup:$PYTHONPATH" .venv/bin/python -m pytest ros2_ws/src/fwr_bringup/test -q
> ```
> 基线：**255 passed, 1.41 s**（2026-11-04）

---

## 3. ruff 查出来的真实问题（待修）

| 文件 | 行 | 问题 | 后果 |
| --- | --- | --- | --- |
| `fwr_bringup/wall_follower.py` | 409-410 | **`twist` 未定义就被使用**（`twist = Twist()` 在 420 行才赋值，而 409 行在前面 `return`） | 一旦进入"倒车恢复"分支就 `NameError`，节点崩 |
| 同上 | 431-434 | 5 个变量赋值后从未使用 | 无害，但说明这段逻辑可能没接上 |

> `wall_follower` 已经不在当前主链路（现在是 `frontier_explorer` / `room_explorer`），
> 所以这个 bug 暂时不影响现场；但**留着是隐患**，建议修掉或整体删除。

其余是未使用的导入（`auto_mapping_preflight.py`、`map_autosave.py`、
`map_free_around_robot.py`、`turn_radius_boost.py`），`ruff --fix` 可安全清理。

---

## 4. 需要 sudo 才能装（本机 sudo 有密码，未安装）

这些是**对本项目最有用**的 ROS 图形工具，装上以后调试会快很多：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rqt-plot \
  ros-humble-rqt-reconfigure \
  ros-humble-rqt-tf-tree \
  ros-humble-rqt-image-view \
  ros-humble-rqt-console \
  ros-humble-plotjuggler-ros \
  ros-humble-foxglove-bridge
```

用途（都对着我们踩过的坑）：

| 工具 | 解决什么 |
| --- | --- |
| `rqt_plot` | 把 `/cmd_vel`、`/odom`、`/scan` 实时画成曲线 —— 我们之前靠临时 Python 脚本量速度，有了它就是一条命令 |
| `rqt_reconfigure` | 运行时改参数（调 `inflation_radius`、`front_stop` 不用改文件重启）|
| `rqt_tf_tree` | 看 `map→odom→base_footprint→laser_front` 链路 —— **TF 滞后那次就是靠这个能一眼看出来** |
| `rqt_console` | 按节点/级别过滤日志（比在终端里滚屏强）|
| `rqt_image_view` | 看 `/camera` 画面 |
| `plotjuggler-ros` | **录一段 bag → 多曲线对比**，做 A/B 参数验证最合适 |
| `foxglove-bridge` | 浏览器里实时看话题/3D/曲线（`http://<ip>:8765`）|

装完先 `source /opt/ros/humble/setup.bash` 再运行。

---

## 5. 已经自带、不用装

`rqt`（主程序）、`rqt_graph`（画节点/话题连接图 —— **排查"指令链断开"最有用**）、
`rqt_bag`（看 bag）、`rviz2`、`nav2_rviz_plugins`、`slam_toolbox`、`plotjuggler` 的 AppImage 版可另行下载。

```bash
source /opt/ros/humble/setup.bash
rqt_graph          # 看 /cmd_vel → 门控 → 碰撞监控 → 选择器 这条链是否真的连着
```
