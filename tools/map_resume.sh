#!/usr/bin/env bash
# 冻结/解冻建图（slam_toolbox 的 Pause 服务是"切换"语义）。
# 发现重影时先跑一次冻结，确认后再跑一次解冻。
set -eo pipefail
ros2 service call /slam_toolbox/pause_new_measurements slam_toolbox/srv/Pause
