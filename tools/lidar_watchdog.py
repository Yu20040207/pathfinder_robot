#!/usr/bin/env python3
"""双雷达看门狗：/scan 或 /scan_rear 断流/静默卡死时，杀掉对应节点让 launch 重新拉起。

2026-11-12 实测：`sllidar_node` 会**静默卡死** —— 进程还在（主线程卡在
`futex_wait_queue`），串口不再产出数据，也不打印任何错误。
2026-11-19 复现：后雷达（A8AB…）跑约 15 分钟后卡死，`/scan_rear` 0 帧，
`ros2 topic hz` 无输出，而进程 `S` 状态、无 dmesg 异常。

因此**不能只靠 `respawn=True`**：进程没退出，launch 不会重启它。
本脚本负责发现"活着但不干活"的节点，`SIGKILL` 掉，交给 launch 的 respawn 拉起。

与旧版（单雷达）的关键区别：
  * 前/后两路分别监控，互不误杀（旧版 `pkill -x sllidar_node` 会把两只一起杀）；
  * 不再自己 `ros2 run` 起节点 —— 那会和 launch 抢串口、造出重复发布者；
    只有在"杀完 6 秒仍无数据"（说明没有 launch 托管）时才兜底自己拉起。

用法：
  .venv/bin/python tools/lidar_watchdog.py
  .venv/bin/python tools/lidar_watchdog.py --stale 3 --cooldown 15 --dry-run
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

_FRONT_PORT = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_'
               'Controller_e00aaf475431f111b7818c5f30d20014-if00-port0')
_REAR_PORT = ('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_'
              'Controller_14da82d39131f111a0eb945f30d20014-if00-port0')

# name, 话题, 节点名（用于匹配 /proc/<pid>/cmdline）, 串口, frame
SPECS = (
    ('front', '/scan', 'sllidar_node', _FRONT_PORT, 'laser_front', None),
    # 2026-11-20：这里原来写的是扁平 ('/scan', '/scan_rear')，
    # 兜底启动时 `for src, dst in state.remaps` 会把字符串当序列解包 →
    # ValueError: too many values to unpack → 看门狗进程崩溃重启（今天 67 次）。
    ('rear', '/scan_rear', 'sllidar_node_rear', _REAR_PORT, 'laser_rear',
     (('/scan', '/scan_rear'),)),
)


def _remap_args(remaps) -> list[tuple[str, str]]:
    """把 remaps 规范成 [(src, dst)]。

    允许两种历史写法：
      * 规范：``(('/scan', '/scan_rear'),)``
      * 旧写法：``('/scan', '/scan_rear')``（扁平两个字符串）
    返回值永远可以直接 `for src, dst in ...`，不会再抛解包错误。
    """
    if not remaps:
        return []
    items = list(remaps)
    if items and all(isinstance(item, str) for item in items):
        items = [tuple(items[i:i + 2]) for i in range(0, len(items) - 1, 2)]
    out: list[tuple[str, str]] = []
    for item in items:
        if isinstance(item, str):
            continue
        try:
            src, dst = item
        except (TypeError, ValueError):
            continue
        out.append((str(src), str(dst)))
    return out


def _matching_pids(node_name: str) -> list[int]:
    """按 cmdline 找雷达驱动进程。

    只认真正的 `sllidar_node` 可执行文件（`.../lib/sllidar_ros2/sllidar_node`），
    避免误杀命令行里恰好含 "sllidar_node" 的 shell/grep/编辑器进程；
    `sllidar_node` 也不能误匹配 `sllidar_node_rear`。
    """
    pids: list[int] = []
    try:
        out = subprocess.run(['pgrep', '-f', 'sllidar_ros2'], capture_output=True,
                             text=True, timeout=5).stdout.split()
    except Exception:                                   # noqa: BLE001
        return pids
    for raw in out:
        try:
            pid = int(raw)
        except ValueError:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as handle:
                cmd = handle.read().replace(b'\0', b' ').decode('utf-8', 'replace')
        except OSError:
            continue
        if '/lib/sllidar_ros2/sllidar_node' not in cmd:
            continue
        if node_name == 'sllidar_node':
            if '__node:=sllidar_node_rear' in cmd:
                continue
        elif f'__node:={node_name}' not in cmd:
            continue
        pids.append(pid)
    return pids


class LidarState:
    def __init__(self, name: str, topic: str, node: str, port: str, frame: str,
                 remaps) -> None:
        self.name = name
        self.topic = topic
        self.node = node
        self.port = port
        self.frame = frame
        self.remaps = remaps
        self.last_msg = 0.0
        self.last_restart = 0.0
        self.seen = False


class Watchdog(Node):
    def __init__(self, stale_s: float, cooldown_s: float, dry_run: bool,
                 clear: bool = True, clear_period_s: float = 0.0):
        super().__init__('lidar_watchdog')
        self._stale = stale_s
        self._cooldown = cooldown_s
        self._dry = dry_run
        self._clear_enabled = clear
        self._clear_period = max(0.0, clear_period_s)
        self._clear_at = 0.0
        self._created = time.monotonic()
        self._clear_clients = []
        self._clear_after: set[str] = set()
        self._states = [LidarState(*spec) for spec in SPECS]

        if clear:
            from nav2_msgs.srv import ClearEntireCostmap
            for name in ('/global_costmap/clear_entirely_global_costmap',
                         '/local_costmap/clear_entirely_local_costmap'):
                self._clear_clients.append(
                    self.create_client(ClearEntireCostmap, name))

        for state in self._states:
            self.create_subscription(
                LaserScan, state.topic,
                (lambda msg, st=state: self._on_scan(msg, st)),
                qos_profile_sensor_data)
        self.create_timer(0.5, self._check)
        self.get_logger().info(
            f'双雷达看门狗启动（{"只报告" if dry_run else "自动重启"}）：'
            f'stale={stale_s}s cooldown={cooldown_s}s '
            f'监控 {", ".join(s.topic for s in self._states)}')

    # ---------- 回调 ----------
    def _on_scan(self, msg, state: LidarState) -> None:
        now = time.monotonic()
        state.last_msg = now
        state.seen = True
        # 驱动阻塞后会把缓冲的旧数据一次性吐出（时间戳落后），也算故障
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            lag = time.time() - stamp
        except Exception:                               # noqa: BLE001
            lag = 0.0
        if lag > 1.5 and now - state.last_restart >= self._cooldown:
            if self._dry:
                self.get_logger().warn(
                    f'{state.topic} 时间戳落后 {lag:.1f}s（dry-run 不重启）')
                return
            self.get_logger().error(
                f'{state.topic} 时间戳落后 {lag:.1f}s（驱动回放旧数据）→ 重启 {state.node}')
            self._restart(state, f'时间戳落后 {lag:.1f}s')
            return
        if state.name in self._clear_after:
            self._clear_after.discard(state.name)
            self._clear_costmaps(f'{state.topic} 恢复')

    # ---------- 巡检 ----------
    def _check(self) -> None:
        now = time.monotonic()
        if self._clear_period > 0 and now - self._clear_at > self._clear_period:
            self._clear_costmaps('周期清空')
        for state in self._states:
            if state.last_msg == 0.0:
                # 从未收到：留 20 s 启动时间（双雷达 + launch 排队）
                if now - self._created < 20.0:
                    continue
                silent = now - self._created
            else:
                silent = now - state.last_msg
                if silent <= self._stale:
                    continue
            if now - state.last_restart < self._cooldown:
                continue
            if self._dry:
                state.last_restart = now
                state.last_msg = now
                self.get_logger().warn(
                    f'{state.topic} 已静默 {silent:.1f}s（dry-run 不重启）')
                continue
            state.last_restart = now
            state.last_msg = now
            self.get_logger().error(
                f'{state.topic} 已静默 {silent:.1f}s → 重启 {state.node}')
            self._restart(state, f'断流 {silent:.1f}s')

    # ---------- 动作 ----------
    def _restart(self, state: LidarState, reason: str) -> None:
        state.last_restart = time.monotonic()
        self._dump_stall_diagnostics(state, reason)
        self._clear_costmaps(f'{state.topic} {reason}')
        self._clear_after.add(state.name)
        pids = _matching_pids(state.node)
        if pids:
            self.get_logger().warn(
                f'杀掉卡死进程 {state.node} pid={pids}（等待 launch respawn）')
            for pid in pids:
                try:
                    subprocess.run(['kill', '-9', str(pid)], timeout=5, check=False)
                except Exception as exc:                # noqa: BLE001
                    self.get_logger().error(f'kill {pid} 失败: {exc}')
        else:
            self.get_logger().warn(f'没找到 {state.node} 进程，直接兜底拉起')
        # 给 launch 的 respawn 一点时间；仍无数据说明没人托管 → 自己起
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            time.sleep(0.5)
            if state.last_msg and time.monotonic() - state.last_msg < 1.0:
                self.get_logger().info(f'{state.topic} 已恢复（launch respawn 生效）')
                return
        self._fallback_start(state)

    def _fallback_start(self, state: LidarState) -> None:
        cmd = ['ros2', 'run', 'sllidar_ros2', 'sllidar_node', '--ros-args',
               '-p', 'channel_type:=serial',
               '-p', f'serial_port:={state.port}',
               '-p', 'serial_baudrate:=460800',
               '-p', f'frame_id:={state.frame}',
               '-p', 'inverted:=false',
               '-p', 'angle_compensate:=true',
               '-p', 'scan_mode:=Standard']
        if state.node != 'sllidar_node':
            cmd += ['-r', f'__node:={state.node}']
        for src, dst in _remap_args(state.remaps):
            cmd += ['-r', f'{src}:={dst}']
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
            self.get_logger().warn(f'{state.node} 6 s 内未被拉起 → 兜底启动')
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().error(f'兜底启动失败: {exc}')

    def _dump_stall_diagnostics(self, state: LidarState, reason: str) -> None:
        """断流瞬间抓一份现场（重启前），用于查"进程在但冻结"的根因。"""
        out = ('/tmp/fwr_runs/lidar_stall_%s_%s.txt'
               % (state.name, time.strftime('%Y%m%d_%H%M%S')))
        try:
            pids = _matching_pids(state.node)
            with open(out, 'w', encoding='utf-8') as handle:
                handle.write('%s: %s at %s\n' % (state.topic, reason, time.ctime()))
                for pid in pids:
                    handle.write('\n=== pid %s ===\n' % pid)
                    for name in ('wchan', 'io', 'status'):
                        path = '/proc/%s/%s' % (pid, name)
                        try:
                            with open(path, 'r', encoding='utf-8',
                                      errors='replace') as f:
                                handle.write('--- %s ---\n%s\n' % (name, f.read()[:2000]))
                        except OSError as exc:
                            handle.write('--- %s 读取失败: %s ---\n' % (name, exc))
            self.get_logger().warn(f'{state.name} 断流现场已保存: {out}')
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f'断流现场保存失败: {exc}')

    def _clear_costmaps(self, reason: str) -> None:
        """清空全局/局部代价地图（异步，不阻塞订阅回调）。"""
        if not self._clear_enabled:
            return
        asked = 0
        for client in self._clear_clients:
            if client.service_is_ready():
                client.call_async(client.srv_type.Request())
                asked += 1
        self._clear_at = time.monotonic()
        if asked:
            self.get_logger().warn(f'清空代价地图（{reason}，{asked} 个服务）')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--stale', type=float, default=2.5, help='静默多少秒判定断流')
    ap.add_argument('--cooldown', type=float, default=15.0, help='同一路重启冷却')
    ap.add_argument('--no-clear', action='store_true', help='不清空代价地图')
    ap.add_argument('--clear-period', type=float, default=0.0,
                    help='每隔多少秒额外清一次代价地图（0=不清）')
    ap.add_argument('--dry-run', action='store_true', help='只报告不重启')
    args = ap.parse_args()

    rclpy.init()
    node = Watchdog(args.stale, args.cooldown, args.dry_run,
                    clear=not args.no_clear, clear_period_s=args.clear_period)
    executor = MultiThreadedExecutor(num_threads=4)
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
