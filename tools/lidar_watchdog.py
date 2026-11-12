#!/usr/bin/env python3
"""雷达看门狗：/scan 断流时自动重启 sllidar_node。

2026-11-12 实测：sllidar_node 在新主控上间歇性**静默卡死** —— 进程还在，
但 CPU 时间与 /proc/<pid>/io 完全冻结，`/scan` 断流；随后 base_bridge 判定
传感器过期、撤销自动授权，frontier_explorer 按安全策略中止任务（"底盘自动
授权丢失"），导航停在中途。

不能直接用 tools/chain_watchdog.py：它是单线程执行器，在生命周期服务调用里
`spin_until_future_complete` 会把 /scan 订阅饿死，反而自己制造断流。

本脚本要点：
  * MultiThreadedExecutor：订阅回调不会被重启流程阻塞；
  * 只做一件事（/scan 新鲜度），不做生命周期转换；
  * 重启后清空基线并冷却 cooldown_s，避免"杀掉→拉起→再杀"循环。

用法：
  .venv/bin/python tools/lidar_watchdog.py
  .venv/bin/python tools/lidar_watchdog.py --stale 2.5 --cooldown 15
  .venv/bin/python tools/lidar_watchdog.py --dry-run
"""
import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.srv import ClearEntireCostmap
from sensor_msgs.msg import LaserScan

BY_ID = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_'
         'Controller_e00aaf475431f111b7818c5f30d20014-if00-port0')
# 断流期间全局代价地图会残留"幽灵障碍"（车身后方永远扫不到，标记清不掉），
# 恢复后规划会报 no valid path。清一次代价地图立刻恢复 —— 所以断流/恢复时各清一次。
CLEAR_SERVICES = (
    '/global_costmap/clear_entirely_global_costmap',
    '/local_costmap/clear_entirely_local_costmap',
)
LAUNCH_ARGS = [
    'ros2', 'run', 'sllidar_ros2', 'sllidar_node', '--ros-args',
    '-p', 'channel_type:=serial',
    '-p', f'serial_port:={BY_ID}',
    '-p', 'serial_baudrate:=460800',
    '-p', 'frame_id:=laser_front',
    '-p', 'inverted:=false',
    '-p', 'angle_compensate:=true',
    '-p', 'scan_mode:=Standard',
]


class LidarWatchdog(Node):
    def __init__(self, stale_s: float, cooldown_s: float, dry_run: bool,
                 clear: bool = True, clear_period_s: float = 0.0):
        super().__init__('lidar_watchdog')
        self._stale = stale_s
        self._cooldown = cooldown_s
        self._dry = dry_run
        self._clear_enabled = clear
        self._clear_period = max(0.0, clear_period_s)
        self._clear_at = 0.0
        self._clear_after_resume = False
        self._clear_clients = [
            self.create_client(ClearEntireCostmap, name) for name in CLEAR_SERVICES]
        self._last_scan = None
        self._created = time.monotonic()
        self._last_restart = 0.0
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(0.5, self._check)
        clear_txt = ('断流/恢复时清空代价地图' if self._clear_enabled else '不清代价地图')
        if self._clear_period > 0:
            clear_txt += f'，每 {self._clear_period:.0f}s 再清一次'
        self.get_logger().info(
            f'雷达看门狗启动（{"只报告" if dry_run else "自动重启"}）：'
            f'stale={stale_s}s cooldown={cooldown_s}s，{clear_txt}')

    def _dump_stall_diagnostics(self, silent_s: float) -> None:
        """断流瞬间抓一份现场（重启前），用于查"进程在但冻结"的根因。

        重点看：wchan（阻塞在哪个内核函数）、io（读写计数是否完全不动）、
        /proc/<pid>/status 的 State/Threads。
        """
        out = '/tmp/fwr_runs/lidar_stall_%s.txt' % time.strftime('%Y%m%d_%H%M%S')
        try:
            pids = subprocess.run(['pgrep', '-x', 'sllidar_node'],
                                  capture_output=True, text=True, timeout=5).stdout.split()
            with open(out, 'w', encoding='utf-8') as handle:
                handle.write('stall %.2f s at %s\n' % (silent_s, time.ctime()))
                for pid in pids:
                    handle.write('\n=== pid %s ===\n' % pid)
                    for name in ('wchan', 'io', 'status', 'cmdline'):
                        path = '/proc/%s/%s' % (pid, name)
                        try:
                            with open(path, 'r', encoding='utf-8', errors='replace') as f:
                                handle.write('--- %s ---\n%s\n' % (name, f.read()[:2000]))
                        except OSError as exc:
                            handle.write('--- %s 读取失败: %s ---\n' % (name, exc))
            self.get_logger().warn(f'断流现场已保存: {out}')
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f'断流现场保存失败: {exc}')

    def _clear_costmaps(self, reason: str) -> None:
        """清空全局/局部代价地图（异步，不阻塞订阅回调）。"""
        if not self._clear_enabled:
            return
        asked = 0
        for client in self._clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
                asked += 1
        self._clear_at = time.monotonic()
        if asked:
            self.get_logger().warn(f'清空代价地图（{reason}，{asked} 个服务）')

    def _on_scan(self, msg) -> None:
        now = time.monotonic()
        self._last_scan = now
        # 2026-11-12：除了"收不到消息"，还要抓"收到的是旧数据"。
        # 现场：驱动阻塞 8~15 s 后把缓冲的消息一次性吐出来，每条都带旧时间戳
        # （collision_monitor 报 "timestamps differ on 8.5 s"）。按到达时间判断会漏掉，
        # 所以这里比较消息头时间戳。
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            lag = time.time() - stamp
            if lag > 1.5:
                if now - self._last_restart >= self._cooldown:
                    self._last_restart = now
                    if not self._dry:
                        self.get_logger().error(
                            f'/scan 时间戳落后 {lag:.1f}s（驱动阻塞后回放旧数据）→ 重启 sllidar_node')
                        self._clear_costmaps(f'/scan 时间戳落后 {lag:.1f}s')
                        self._clear_after_resume = True
                        self._restart()
                return
            self._stamp_ok = True
        except Exception:                                   # noqa: BLE001
            pass
        if self._clear_after_resume:
            self._clear_after_resume = False
            self._clear_costmaps('雷达恢复')

    def _check(self) -> None:
        now = time.monotonic()
        if self._clear_period > 0 and now - self._clear_at > self._clear_period:
            self._clear_costmaps('周期清空')
        if self._last_scan is None:
            # 从未收到过：留出 10 秒启动时间，避免开机时误杀
            if now - self._created < 10.0:
                return
            silent = now - self._created
        else:
            silent = now - self._last_scan
            if silent <= self._stale:
                return
        if now - self._last_restart < self._cooldown:
            return
        self._last_restart = now
        self._last_scan = now
        if self._dry:
            self.get_logger().warn(f'/scan 已静默 {silent:.1f}s（dry-run 不重启）')
            return
        self.get_logger().error(f'/scan 已静默 {silent:.1f}s → 重启 sllidar_node')
        if not self._dry:
            self._dump_stall_diagnostics(silent)
        self._clear_costmaps(f'/scan 断流 {silent:.1f}s')
        self._clear_after_resume = True
        self._restart()

    def _restart(self) -> None:
        try:
            subprocess.run(['pkill', '-x', 'sllidar_node'], timeout=5,
                           check=False)
            time.sleep(1.0)
            subprocess.Popen(LAUNCH_ARGS, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
            self.get_logger().warn('已重新拉起 sllidar_node')
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f'重启雷达失败: {exc}')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--stale', type=float, default=2.0,
                    help='/scan 静默超过该秒数就重启（默认 2.0）')
    ap.add_argument('--cooldown', type=float, default=15.0,
                    help='两次重启之间的最小间隔（默认 15）')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--no-clear', action='store_true',
                    help='断流/恢复时不自动清空代价地图')
    ap.add_argument('--clear-period', type=float, default=0.0,
                    help='额外每隔该秒数清一次代价地图（0=只在断流相关时刻清）')
    args = ap.parse_args()
    rclpy.init()
    node = LidarWatchdog(args.stale, args.cooldown, args.dry_run,
                         clear=not args.no_clear, clear_period_s=args.clear_period)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
