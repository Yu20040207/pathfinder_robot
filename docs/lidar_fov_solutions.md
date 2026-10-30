# 雷达视场受限（后方盲区）的开源解决方案调研

> 现状：RPLIDAR C1 装在车头，**只能看到前方与两侧（约 180°）**，后方被车身结构挡住。
> 建图时后方永远是未知区，转弯/闭环时缺少约束，是当前建图质量的瓶颈之一。

## 0. 先说权威结论（避免走弯路）

slam_toolbox 维护者 **Steve Macenski** 在官方 issue 里的原话：

> "The lidar graph slam is suited for ~180 deg lidar views or higher.
> ~120 I think would probably be fine too, but that's starting to push it."

> "the experiments with an empty simulation room aren't really valid…
> Matcher based approaches aren't really suited for low-visibility."

出处：
- https://github.com/SteveMacenski/slam_toolbox/issues/239#issuecomment-2091954937
- https://github.com/SteveMacenski/slam_toolbox/issues/695

**两条结论**：
1. **180° 正好在图优化 SLAM 的可用下限**——能用，但没有余量。
2. 空房间漂移是匹配式 SLAM 的固有局限，换算法也难根治。

同一 issue 里给出的调参建议（低 FOV 场景）：

```yaml
minimum_travel_distance: 0.15   # 更频繁地加节点/匹配（费 CPU）
minimum_travel_heading: 0.2
loop_search_maximum_distance: 5.0
correlation_search_space_smear_deviation: 0.1
```

> ⚠️ 我们当前是 `minimum_travel_distance: 0.5`（为了绕开 slam_toolbox 崩溃 issue #642），
> 代价就是匹配次数少、更容易漂。**可以试 0.25~0.3 折中**。

## 1. 可选方案总览（按性价比排序）

| # | 方案 | 成本 | 补盲效果 | 成熟度 | 适合我们吗 |
|---|---|---|---|---|---|
| **A** | 抬高/移位雷达 | **0 元** | 看挡的是什么 | — | ⭐先试 |
| **B** | 深度相机 → 虚拟激光扫描 | **0 元**（相机已有） | 中（3~4 m 量程） | ROS 官方包 | ⭐⭐ 推荐 |
| **C** | 加一路后向 2D 雷达 + 双雷达融合 | ~300-600 元 | 高（真正 360°） | 社区包，工业常用 | ⭐⭐⭐ 最优 |
| **D** | 直接换 360° 雷达 | 800-2000 元 | 彻底解决 | 官方支持 | 预算允许时 |
| **E** | 纯策略补偿（不改硬件） | 0 元 | 有限 | — | 现在就在用 |

---

## 2. 方案 B：深度相机转虚拟激光扫描（**包已经装好了**）

**包**：`depthimage_to_laserscan`（ROS 2 官方，**本机已安装 2.5.1**）

```bash
apt-cache policy ros-humble-depthimage-to-laserscan   # 已安装
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --help
```

**原理**：把深度图的某一水平行（`scan_height`）投影成 `sensor_msgs/LaserScan`，
输出和真实雷达完全一样的消息类型 → 可以喂给任何 SLAM/代价地图。

**关键参数**：

| 参数 | 作用 |
|---|---|
| `output_frame` | 虚拟扫描的坐标系（设为相机 frame） |
| `scan_height` | 取深度图哪一行（默认 1，对应相机光轴高度） |
| `range_min` / `range_max` | 量程（HP60C 约 0.3~4 m） |

**限制**：深度相机量程短（3~4 m）、FOV 约 60~90°，替代不了雷达，但**能补上雷达看不到的近处盲区**。

## 3. 方案 C：双雷达融合（真正的 360°）

两个可用的开源包：

### `ros2_laser_scan_merger`（社区，参数化最好）
https://github.com/mich1342/ros2_laser_scan_merger

- 输入多路 `LaserScan`，按各自相对位姿/朝向拼成一个**虚拟 360° 扫描**
- 每路单独配置：`x/y/z`、`yaw`、`inverted`
- ROS 2 原生包，Humble 可用

### `ira_laser_tools`（老牌，ROS 1 移植）
https://github.com/fravizel-dev/ira_laser_tools （ROS 2 foxy 移植）
原始 ROS 1 版：http://wiki.ros.org/ira_laser_tools

- `laserscan_multi_merger`：把多路激光拼成一路
- 工业界大量在用（AGV 前后双雷达的标准做法）

**硬件建议**：再加一台 **RPLIDAR C1**（与现有一致，驱动直接复用），
装在后部朝后，和现有雷达**共用一个 launch**，输出 `/scan_merged` 给 SLAM。

> 注意：两雷达同型号同频（10 Hz）时容易互相干扰（都是 905 nm 波段），
> 实际部署时建议**错开安装高度**或选不同型号（如 S2 + C1）。

## 4. 方案 D：换 360° 雷达

直接上 RPLIDAR A2M12 / S2 / S3，一次解决：

- 优点：不用融合、不用标定、SLAM 效果最好
- 缺点：成本，且需要重新设计安装（现在的位置被结构挡住，360° 雷达装那儿也会被挡）

## 5. 方案 E：不改硬件的补偿策略（我们现在用的）

| 做法 | 效果 |
|---|---|
| 每个位置**先转 180° 再前进** | 让后方区域被扫到一次 |
| 贴墙走、墙角停 2 s | 用几何约束代替视场 |
| 绕回起点闭环 | 事后用位姿图纠正 |
| 空房间临时放个箱子 | 制造特征 |

---

## 6. 给本项目的建议路线

```
第一步（0 元，今天就能试）
  · 检查后方到底被什么挡住 —— 拍张照片看雷达是否只是被线束/支架挡了一点
  · 抬高雷达 5~10 cm 试试（可能直接看到后方）

第二步（0 元，已有硬件）
  · 用 HP60C 深度相机跑 depthimage_to_laserscan，补后向近处盲区
  · 单独验证：虚拟扫描能否稳定输出、和雷达数据是否冲突

第三步（300~600 元，一劳永逸）
  · 加一台后向 C1/A1 + ros2_laser_scan_merger 合成 360°
  · SLAM、Nav2、碰撞监控全部直接受益（后方也有避障了）
```

## 7. 参考链接汇总

| 主题 | 链接 |
|---|---|
| slam_toolbox 低 FOV 讨论（维护者原话） | https://github.com/SteveMacenski/slam_toolbox/issues/239 |
| slam_toolbox 低 FOV 复现 | https://github.com/SteveMacenski/slam_toolbox/issues/695 |
| depthimage_to_laserscan（官方，已装） | https://index.ros.org/p/depthimage_to_laserscan/ |
| 双雷达融合 `ros2_laser_scan_merger` | https://github.com/mich1342/ros2_laser_scan_merger |
| 双雷达融合 `ira_laser_tools`（ROS 2 移植） | https://github.com/fravizel-dev/ira_laser_tools |
| 原始 ira_laser_tools 文档 | http://wiki.ros.org/ira_laser_tools |
| 激光滤波（裁剪/范围过滤） | `ros-humble-laser-filters`（apt 可装） |
