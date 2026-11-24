"""常驻硬件层（手动模式）：底盘/雷达×2/IMU/EKF/扫描合并/车尾守卫。

2026-11-24 用户要求：开机先起来就是**手动模式**，只有切换模式时才启动对应栈；
切模式时先把现有模式清理干净。为此把"硬件"和"算法"分开：

  fwr-base.service   → 本 launch（一直活着，手动模式靠它）
  fwr-nav.service    → real_navigation.launch.py start_base:=false
  fwr-map.service    → real_mapping.launch.py  start_base:=false start_slam:=true

这样 `base_bridge` 只被起一份（串口不被抢），开机即可手动开车。
本文件只是 `real_mapping.launch.py start_slam:=false` 的薄包装，
节点定义只有一处，不做复制。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory('fwr_bringup'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / 'launch' / 'real_mapping.launch.py')),
            launch_arguments={
                'start_slam': 'false',       # 不起 slam_toolbox / map_view / save
                'start_base': 'true',        # 只起硬件层
                'start_ultrasonic': 'false', # 车尾用后雷达，超声默认不开
            }.items()),
    ])
