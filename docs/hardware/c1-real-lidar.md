# C1 实机雷达接入

已在 Ubuntu 22.04 / ROS 2 Humble 上验证 SLAMTEC RPLIDAR C1。官方 ROS 2 驱动为
`https://github.com/Slamtec/sllidar_ros2`，测试版本固定为
`34300099fadfc772965962dec837bf436706188f`（BSD-2-Clause）。驱动源码位于
`ros2_ws/src/sllidar_ros2`，作为外部依赖，不纳入本仓库提交。

首次安装：

```bash
cd /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
git -C sllidar_ros2 checkout 34300099fadfc772965962dec837bf436706188f
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sllidar_ros2 fwr_bringup
```

当前电脑的稳定串口名可通过 `ls -l /dev/serial/by-id/` 查看。运行前，账户需要具备
串口读写权限，例如加入 `dialout` 组并重新登录；测试时曾通过 `setfacl` 临时授权，
重插设备后该权限可能消失。运行时使用与 Gazebo 不同的 ROS 域：

```bash
source /opt/ros/humble/setup.bash
source /home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=94
ros2 launch fwr_bringup lidar_c1_real.launch.py serial_port:=/dev/ttyUSB0
```

RViz 固定坐标为 `base_link`，显示 `/scan`、TF 和底盘模型。扫描消息坐标为
`laser_front`。实机前方遮挡对照测试表明，雷达原始扫描的 180° 指向底盘前方，
因此 URDF 将 `laser_front` 相对 `base_link` 绕 Z 轴旋转 180°。2026-10-22
用户实测扫描中心距底盘最前沿向后 50 mm、左右居中、离地 75 mm。
底盘长 520 mm，故扫描中心相对底盘几何中心向前 210 mm；
`base_link` 离地 32.5 mm，因此当前固定关节为
`xyz=(0.21, 0, 0.0425) m`，从 `base_footprint` 计算扫描中心离地 75 mm。
底盘外壳高度仍为示意值；须通过实机 `/scan` 核查车体和线束在这一低安装高度造成的遮挡。
静止采样 20 帧时，车头 45° 扇区没有 0.35 m 内近点，车尾 45° 扇区则有大量约 0.1 m 的近点；需要分辨车体/线束遮挡与现场近物，不能把车尾扫描当作完整后向避障依据。
