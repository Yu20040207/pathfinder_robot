# 前进与受限倒车的方向相关近障保护设计

日期：2026-10-21。范围：既有自动建图运动链路；先做仿真，实车自动倒车保持关闭。

## 目标与依据

持续前方障碍触发 Humble Collision Monitor 的固定 `front_stop` 后，受限回退控制器已发出负速度，自动命令选择器也已选中负速度，但安全出口持续为零。仿真证据：`/tmp/fwr_auto_sim_hjyf3yq1.log`；脚本记录 `reverse_raw=8`、`reverse_unchecked=16`、`reverse_commands=0`、`reverse_distance=0`。目标是让机器人在已批准的受限回退条件全部满足时，能远离车前障碍，同时继续检查车后可见区域和所有现有停止条件。

Nav2 新版 `VelocityPolygon` 能按命令方向切换保护区；本机 `ros-humble-nav2-collision-monitor` 1.1.20 的已安装头文件没有这个类型。因此在当前 Humble 上复用两个独立 Collision Monitor 实例，不升级整套 Nav2，也不允许回退命令绕开碰撞监控。

参考：

- [Nav2 Collision Monitor 源码与方向相关多边形](https://github.com/ros-navigation/navigation2/blob/main/nav2_collision_monitor/src/collision_monitor_node.cpp)
- [Nav2 Collision Monitor 官方示例](https://github.com/ros-navigation/navigation2/blob/main/nav2_collision_monitor/params/collision_monitor_params.yaml)
- [本机 Humble 对应源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/collision_monitor_node.cpp)

## 消息流与组件

现有 `auto_command_gate` 保持导航和回退互斥，并发布 `/cmd_vel_auto_unchecked`。两个 Nav2 Collision Monitor 同时订阅这一输入及真实 `/scan`，分别输出 `/cmd_vel_auto_forward_filtered` 与 `/cmd_vel_auto_reverse_filtered`。前向实例只配置车前停止区与减速区；后向实例只配置侧后可见停止区。新增 `directional_safety_selector` 是唯一发布 `/cmd_vel_auto_safe` 的节点；底盘桥接和仿真底盘只消费这个安全出口。人工键盘控制仍走现有人工入口。

选择器根据**当前未经滤波的命令方向**选择对应监控输出：正线速度取前向实例，负线速度取后向实例。零线速度且有角速度时，必须同时取得前后两个监控在本次输入之后发布的新鲜结果，且两者都允许同向角速度；输出其中绝对值较小的角速度。任一路停止或过期即归零；全零命令输出零。选择器不得依据某个监控的输出符号决定方向，以免前方停车输出零后错误选择另一条链。此原地转向补充于 2026-10-21 获用户确认；原因是 Nav2 起始航向调整需要原地旋转，原先全部归零会使导航无法起步。

两个监控必须使用不同节点名、生命周期管理和输出 topic。若所需实例未激活、输出不新鲜、传感器或探索状态不新鲜、方向变化后尚未收到该方向的新滤波结果，选择器持续输出零。选择器退出或链路中断时底盘桥接看门狗仍负责撤销自动授权。`/cmd_vel_auto_safe` 不得有第二个发布者。

## 方向切换与消息关联

Humble Collision Monitor 输入/输出是 `Twist`，没有序列号。选择器必须在每次零↔前进↔倒车转换时清空两路缓存，先发布零速；只有在方向切换之后，接收到与该方向新输入相匹配的滤波输出，才放行。匹配至少要求：输出与输入同向或为零、线速度不超过输入幅值、角速度不超过输入幅值，且两者到达间隔位于短新鲜度窗口内。无法可靠配对的结果只能使机器人保持停止，不得回退使用旧缓存。

底盘原有自动授权、故障锁定、串口 300 ms 超时、回退许可、IMU/编码器与地图轨迹检查、单段 0.10 m 和总计 0.30 m 上限都保留。人工接管、Nav2 停止、雷达断流、地图或局部代价图过期、任一必要节点退出时归零。近障监控只处理**当前激光可见区域**；真实车尾盲区仍不能由前置雷达证明安全。未经现场有人看护、车尾全程可见和实测制动距离验证，实车回退许可保持关闭。

## 仿真验收

1. 单元测试覆盖前进、倒车、零速、切换期间零速、旧滤波包、丢包、反号、超幅、雷达过期、监控实例过期、人工停止。
2. 静态 ROS 测试核对双实例 topic、生命周期和唯一安全出口；断开任一被选实例后，安全出口在既有命令超时窗口内归零。
3. 持续车前障碍场景：前进指令被前向实例停止；Nav2 判定路径不可达且最近轨迹已知空闲时，回退原始负速度经后向实例到达安全出口。实测倒车位移须大于 0 且不超过 0.10 m 的单段上限；随后停止并记录编码器、IMU、地图和故障码。仅进入 `reversing` 状态不算通过。
4. 侧后障碍、雷达断流、后向实例退出、方向快速切换、后方轨迹未知或占据、定位跳变均必须停机，不能产生负速度安全输出。
5. 前向有足够通道的持续障碍绕行回归仍应通过；此前固定目标侧向绕行证据不能代替新链路的回归结果。

仿真通过后才进入实车静态预检；随后在封闭有人看护的场地依次做单个前进目标、障碍前停车与绕行、单段最多 0.10 m 的倒车。倒车实车验收之前，小范围自动建图可以只在禁用倒车的条件下分阶段评估，不能宣称自主脱困完成。
