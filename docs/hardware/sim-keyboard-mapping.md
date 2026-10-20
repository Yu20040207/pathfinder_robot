# 走廊仿真键盘建图

此流程使用独立的 `ROS_DOMAIN_ID=166`。键盘命令只送到 Gazebo 的 `/cmd_vel`，不连接实车底盘桥接。

终端一启动 Gazebo、SLAM 和 RViz：

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=166
ros2 launch fwr_bringup corridor_sim.launch.py headless:=true mapping:=true rviz:=true
```

终端二启动键盘控制，保持此终端处于焦点：

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot
./scripts/start_sim_keyboard.sh
```

`W/S` 前进和后退，`A/D` 原地左转和右转，空格停机，`Q` 退出。长按按键产生连续指令；松开后约 0.18 秒停机。退出脚本会再向仿真发送零速度，并恢复原来的键盘重复设置。结束仿真时在终端一按 `Ctrl+C`。

当前仿真模型采用 65 mm 驱动轮和 263 mm 轮距。Gazebo 只能验证 ROS 指令链、运动方向及停机逻辑；实车左轮倒转时的失速仍需要后续单独排查。
