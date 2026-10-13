# 二代底盘走廊仿真与导航

该阶段在 Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress 上验证底盘差速运动、C1 安装位置、激光扫描、建图与 Nav2 路径规划。场景和底盘尺寸为设计预览，不代表实物已完成标定；C1 仿真扫描参数目前用于链路测试。

## 启动

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fwr_bringup fwr_rviz_nav --symlink-install
source install/setup.bash
ros2 launch fwr_bringup corridor_sim.launch.py
```

无图形界面时加 `headless:=true`；启动在线建图时加 `mapping:=true`；启动已知地图上的 Nav2 导航时加 `navigation:=true`；打开 RViz 时加 `rviz:=true`。`mapping` 与 `navigation` 分别使用在线 SLAM 和静态地图，不要同时启用。场景包含 2.7 米宽走廊和一个障碍物；机器人是两驱动轮、两前随动轮的简化模型，C1 位于底盘顶部前端。Gazebo 车身外观复用 RViz 的圆角 STL，车身透明度和轮子、随动轮、雷达的颜色与预览一致；物理碰撞仍用简单几何体，随动轮的转向机构暂未仿真。ROS 话题包括 `/clock`、`/scan`、`/odom`、`/tf`、`/joint_states` 和 `/cmd_vel`；建图或导航模式另有 `/map` 和 `map → odom` TF。RViz 中的红点是二维 LaserScan 测距点，并非三维 PointCloud2。

## 路径规划与避障验收

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fwr_bringup corridor_sim.launch.py navigation:=true rviz:=true
```

RViz 的 Fixed Frame 是 `map`。在工具栏选择 **Publish Point**，依次点击地图中的起点与终点。右侧“两点导航”面板会显示坐标，绿色为起点、红色为终点，绿色线条只预览起点到终点这一段。路径规划成功后点击 **开始导航**；面板先向 `/navigate_to_pose` 发送起点，**收到起点成功结果后**才发送终点。机器人从实际当前位置出发，不会瞬移；当前位置到起点这一段会在导航时由 Nav2 规划，若该段不可通行，不会发送终点任务。点击第三次会重新选择起点，“清除两点”可重来。也可用原有 `Nav2 Goal` 工具直接指定单个目标。导航目标可选障碍物后方的 `(4.5, 0.45)`。仿真使用随项目提供的已知走廊地图 `maps/corridor_nav.yaml` 做全局规划，并用 `/scan` 更新局部代价地图。示例路径从 `(0, 0)` 出发，绕过 `(3.2, -0.45)` 的障碍物后到达目标；动作结果 `GoalStatus.STATUS_SUCCEEDED` 的数值是 **4**。可检查 `/two_point_preview_path`、`/two_point_markers`、`/plan`、`/local_costmap/costmap`、`/odom` 和 `/scan`。

当前仿真已验证从起点绕过固定障碍物并到达前方目标；两点途经导航仍需按实际选点和动态障碍情况检查。在线建图与导航是两个单独模式。在线 SLAM 在这条重复纹理走廊中运动时可能漂移，故导航验收使用与仿真场景匹配的静态地图。导航模式还会桥接 `/world/corridor/clock` 到 ROS `/clock`，并用 Gazebo 世界位姿和 `/odom` 动态计算 `map → odom`；这样在 Gazebo 界面移动机器人后，激光点仍能落在地图正确位置。这个世界位姿校正仅供仿真使用，实机必须接入真正的定位，并重新标定轮距、轮径、雷达位姿及机器人碰撞外形。动态障碍避让和急停仍需单独实测。

不要同时启动两套 `corridor_sim.launch.py`。若 RViz 提示规划失败，先确认 `/clock`、`/map`、`/scan` 都在发布，且 `map → odom → base_link` 变换存在；地图上的绿色预览起点和红色终点也应位于可通行区域。Gazebo 中手动添加障碍物可能堵住走廊，需从全局代价地图判断，而不能只看灰色静态地图。

## 仿真倒车脱困

走廊仿真专用 Nav2 使用 Regulated Pure Pursuit 跟踪路径：遇到大转角先对准路径，再低速前进并预测前方碰撞；它本身只向前行驶。行为树以 2 Hz 检查当前全局路径：路径有效时保留已选障碍物一侧，路径被新障碍挡住或目标改变时重规划。全局代价地图仍以 3 Hz 更新。全局规划器改用 Smac 2D，以路程和障碍代价共同选路，较宽的净空可抵消小幅路程差；路径仍有效时保持现有路线，避免仅因左右路线代价接近而来回换边。这是加权取舍，不保证数学意义上的绝对最宽路线。控制器仍会独立检查前方碰撞并停车；重规划频率不是保证畅通或保证固定反应距离的参数。恢复行为树在规划或跟踪失败后优先尝试带碰撞检查的 0.45 m 倒车，再清理代价地图、原地转向或等待。如果后方代价地图显示障碍，倒车恢复会被拒绝。两点面板只选择位置，故仿真目标的朝向容差设为约 180°；如果任务需要精确朝向（例如对准电梯按键），必须另行配置目标朝向约束。绿色预览线只展示选定起点到终点的规划段，实际行驶路径由 Nav2 单独计算。

手工添加圆柱后，走廊中的连续急弯可能让局部控制器预测到碰撞并触发恢复；全球路径显示为绿色并不保证机器人能够以当前尺寸和转向轨迹穿过。测试时可检查 `/local_costmap/costmap`、`/cmd_vel` 和 Nav2 日志中的 `detected collision ahead`。仿真代价地图现在使用 0.60 × 0.43 m 矩形外形，覆盖 0.52 × 0.35 m 车身碰撞盒并预留约 4 cm/侧，控制器运行频率为 10 Hz；在两圆障碍之间约 0.74 m 净空处实测到达目标。该外形仅适用于当前仿真底盘，实机必须重新测量完整扫掠外形和安全余量，并验证转弯、定位误差及动态障碍下的净空。

上述倒车配置只供 Gazebo 仿真使用。实体样机目前规划的是底盘前置单雷达，后方存在盲区；在后向感知、制动、轮速反馈和急停验收完成前，不把该倒车配置用于实机自主导航。

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /map --once  # 仅 mapping:=true 时
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}}'
```

在手动发布速度之后，必须再发布零速度停止；当前 Gazebo 差速插件未配置通信超时停车，不能用它证明实机安全逻辑。实机 ESP32-S3 仍须独立实现通信超时停车和急停。

后续迁移到 Jetson Orin Nano Super 8GB 时，保留 ROS 话题与 TF 接口；仿真场景可继续在开发电脑运行，Jetson 端需按实际 JetPack、ROS 发行版和 ARM64 驱动重新验证。
