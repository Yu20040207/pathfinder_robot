#!/usr/bin/env python3
"""链路看门狗：碰撞监控卡死时自动重启它。

2026-10-30 实测：Nav2 的 collision_monitor 在触发一次 `stop`（front_stop 多边形）
之后会进入“不再处理指令”的状态 —— 输入 /cmd_vel_auto_unchecked 仍有 20 Hz，
但输出 /cmd_vel_auto_forward_filtered 彻底静默，于是
`/cmd_vel_auto_safe` 恒为 0，表现为「点了开始导航，车一动不动」。
手动 `ros2 lifecycle set <monitor> deactivate/activate` 立刻恢复。

用法：
  .venv/bin/python tools/chain_watchdog.py            # 自动模式
  .venv/bin/python tools/chain_watchdog.py --dry-run  # 只报告不操作

判据：连续 2 秒「输入有 20 Hz、输出一帧都没有」→ 重启两个监控。
"""
import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import TransitionCallbackReturn  # noqa: F401  (类型存在性检查)
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


IN_TOPIC = '/cmd_vel_auto_unchecked'
OUT_TOPIC = '/cmd_vel_auto_forward_filtered'
MONITORS = ('collision_monitor_forward',)


class Watchdog(Node):
    def __init__(self, dry_run: bool):
        super().__init__('chain_watchdog')
        self._dry = dry_run
        self._started = time.monotonic()
        # 启动宽限期：栈起来后监控要几十秒才稳定发布
        self._grace_s = float(os.environ.get('FWR_WATCHDOG_GRACE_S', '45'))
        self._in = 0
        self._out = 0
        self._last_in = 0.0
        self._last_out = 0.0
        # 2026-11-02：雷达断流会让局部代价地图"没有障碍数据"，控制器连续 abort、
        # Nav2 目标失败（现场实测断流 19.8 s 后任务直接结束）。
        # 这里监控 /scan，超过 scan_stale_s 没数据就重启雷达节点。
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data
        self.declare_parameter('scan_stale_s', 2.0)
        # 2026-11-02：默认**只告警、不自动重启**。
        # 实测自动重启会陷入'杀掉→拉不起来→再杀'的循环，比不重启更糟。
        # 需要自动重启时显式传 restart_scan:=true。
        self.declare_parameter('restart_scan', False)
        # 2026-11-02：/scan 多发布者检测。今天两个 sllidar_node 抢同一串口，
        # launch 里那个抛 RCLError 崩溃，/scan 出现异常数据 → 局部代价地图坏掉
        # → 控制器每帧 abort → 车不动，排查了半天。
        # 默认**只告警并列出节点名**，不自动杀 —— 盲杀可能杀掉正常的那个。
        self.declare_parameter('dedupe_scan', False)
        self._last_scan = 0.0
        self._scan_n = 0
        self._in_nz = 0
        self._out_nz = 0
        self._blocked_pts = 0
        self._zero_run = 0.0
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, IN_TOPIC, self._on_in, 20)
        self.create_subscription(Twist, OUT_TOPIC, self._on_out, 20)
        self._change_clients = {
            m: self.create_client(ChangeState, f'/{m}/change_state') for m in MONITORS
        }
        self._state_clients = {
            m: self.create_client(GetState, f'/{m}/get_state') for m in MONITORS
        }
        self.create_timer(1.0, self._check)
        self.declare_parameter('out_silent_s', 3.0)
        self.create_timer(5.0, self._health)
        self.get_logger().info(
            f'看门狗启动（{"只报告" if dry_run else "自动重启"}）：{IN_TOPIC} → {OUT_TOPIC}')

    def _on_in(self, msg):
        self._in += 1
        self._last_in = time.monotonic()
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self._in_nz += 1

    def _on_out(self, msg):
        self._out += 1
        self._last_out = time.monotonic()
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self._out_nz += 1

    def _state_of(self, name):
        """用 CLI 查状态。

        2026-11-18 现场：用 rclpy 的 GetState 客户端查询一直返回 'unknown'
        （"collision_monitor_forward 状态 unknown"），于是 _restart 里所有
        transition 都失败，看门狗触发 36 次一次没修好。CLI 反而稳定可用。
        """
        try:
            out = subprocess.run(
                ['ros2', 'lifecycle', 'get', '/' + name],
                capture_output=True, text=True, timeout=10,
                env=dict(os.environ)).stdout.strip()
        except Exception:
            return 'unknown'
        for st in ('active', 'inactive', 'unconfigured', 'finalized'):
            if st in out:
                return st
        return 'unknown'

    def _cli(self, name, verb):
        try:
            r = subprocess.run(
                ['ros2', 'lifecycle', 'set', '/' + name, verb],
                capture_output=True, text=True, timeout=20,
                env=dict(os.environ))
            return 'successful' in (r.stdout + r.stderr)
        except Exception:
            return False

    def _restart(self, name):
        """不看状态，直接"停->起"，然后用输出是否恢复来判定。

        2026-11-18：这个看门狗原来先查状态再决定怎么转，但 rclpy 和 CLI 两种
        查法在现场都会返回 'unknown'（ros2 lifecycle get 10 s 超时），于是
        36 次重启全部失败、还要反复打扰监控。现在干脆不查 —— deactivate 失败
        也无所谓（可能已经 inactive），关键是后面那次 activate，然后用
        "输出是否恢复"验证效果。
        """
        self.get_logger().warn(f'{name} 输出静默 → deactivate/activate')
        self._cli(name, 'deactivate')
        time.sleep(1.5)
        ok = self._cli(name, 'activate')
        time.sleep(1.5)
        last_out = getattr(self, '_last_out', 0.0)
        back = bool(last_out) and (time.monotonic() - last_out) < 2.0
        self.get_logger().info(
            f'{name} activate={"成功" if ok else "失败"}，输出'
            f'{"已恢复" if back else "仍静默"}')
        if not back:
            self.get_logger().error(
                f'{name} 无法恢复 → 需要重启导航栈（bash /tmp/restart_nav.sh）')

    def _scan_publishers(self):
        try:
            infos = self.get_publishers_info_by_topic('/scan')
            return [i.node_name for i in infos]
        except Exception:                          # noqa: BLE001
            return []

    def _on_scan(self, msg) -> None:
        # 前停区里是否有真实障碍点（判断"监控压零"是不是真的该停）。
        # 前停区在 base 系是 x∈[0.32,0.46]、|y|<=0.22；
        # 雷达装在 base 前方 0.21 m，所以换算到 laser 系要减 0.21。
        n = 0
        for i, r in enumerate(msg.ranges):
            if not (math.isfinite(r) and msg.range_min <= r <= msg.range_max):
                continue
            a = msg.angle_min + i * msg.angle_increment
            fx = -r * math.cos(a)      # /scan: 0°=车尾，±180°=车头
            ly = r * math.sin(a)
            if 0.11 <= fx <= 0.25 and abs(ly) <= 0.22 and r > 0.05:
                n += 1
        self._blocked_pts = n
        self._last_scan = time.monotonic()
        self._scan_n += 1

    def _restart_scan(self) -> None:
        """重启雷达节点（按 by-id 串口，避免 /dev/ttyUSB* 重枚举）。"""
        import subprocess
        try:
            subprocess.run(['pkill', '-f', 'sllidar_node'], timeout=5,
                           check=False)
            time.sleep(1.0)
            by_id = ('/dev/serial/by-id/'
                     'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                     'e00aaf475431f111b7818c5f30d20014-if00-port0')
            # 2026-11-02：必须带齐参数！只传 serial_port 会让节点在默认
            # 波特率下打不开串口、起来就退出 —— 现场表现为"看门狗把雷达杀了
            # 却拉不起来，/scan 直接 0 Hz"。参数与 real_mapping.launch.py 一致。
            subprocess.Popen(
                ['ros2', 'run', 'sllidar_ros2', 'sllidar_node', '--ros-args',
                 '-p', 'channel_type:=serial',
                 '-p', f'serial_port:={by_id}',
                 '-p', 'serial_baudrate:=460800',
                 '-p', 'frame_id:=laser_front',
                 '-p', 'inverted:=false',
                 '-p', 'angle_compensate:=true',
                 '-p', 'scan_mode:=Standard'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self.get_logger().warn('已重新拉起雷达节点')
        except Exception as exc:                     # noqa: BLE001
            self.get_logger().error(f'重启雷达失败: {exc}')

    def _health(self):
        """周期健康检查：状态不对、或**输出静默**就恢复。

        2026-11-01：原来只查状态和"输入有、输出无"。现场遇到"输入也在发、
        但监控完全不回复"的情况（前端空旷也输出 0），两个条件都没覆盖到。
        现在只要**指令在流动而监控输出静默超过 out_silent_s** 就重启。
        """
        if self._dry:
            return
        now = time.monotonic()
        cmd_flowing = (now - self._last_in) < 1.0
        out_silent = (now - self._last_out) > float(
            self.get_parameter('out_silent_s').value)
        # /scan 多发布者检查（今天的事故源）
        pubs = self._scan_publishers()
        if len(pubs) > 1:
            if (now - getattr(self, '_last_dup_warn', 0.0)) > 30.0:
                self._last_dup_warn = now
                self.get_logger().error(
                    '/scan 有 %d 个发布者 %s —— 多个雷达节点抢串口会让'
                    '代价地图数据错乱、控制器连续 abort。' % (len(pubs), pubs))
                if bool(self.get_parameter('dedupe_scan').value):
                    import subprocess
                    subprocess.run(['pkill', '-f', 'sllidar_ros2/lib'],
                                   check=False)
                    self.get_logger().warn('已结束全部雷达进程，请重启一个')

        # 雷达断流检查（优先级最高：没有它一切都白搭）
        stale = float(self.get_parameter('scan_stale_s').value)
        if self._last_scan > 0 and (now - self._last_scan) > stale:
            if bool(self.get_parameter('restart_scan').value) and \
                    (now - getattr(self, '_last_scan_restart', 0.0)) > 10.0:
                self.get_logger().error(
                    '/scan 已静默 %.1fs → 重启雷达' % (now - self._last_scan))
                self._last_scan_restart = now
                self._restart_scan()
            self._last_scan = now      # 避免每轮都触发

        for name in MONITORS:
            state = self._state_of(name)
            if state not in ('active', 'unknown'):
                self.get_logger().warn('%s 处于 %s，自动恢复' % (name, state))
                self._restart(name)
            elif cmd_flowing and out_silent:
                self.get_logger().warn(
                    '%s 指令在流动但输出静默 %.1fs，重启'
                    % (name, now - self._last_out))
                self._restart(name)

    def _check(self) -> None:
        now = time.monotonic()
        # 2026-11-18：启动宽限期。导航栈刚起来时监控还没发布，
        # 看门狗会立刻判"卡死"并 deactivate/activate —— 而这恰恰是
        # collision_monitor "configure 超时/卡在 inactive" 的根源。
        if now - self._started < self._grace_s:
            self._in = self._out = 0
            return
        in_alive = now - self._last_in < 1.0
        out_alive = now - self._last_out < 1.0
        in_n, out_n = self._in, self._out
        self._in = self._out = 0
        in_nz, out_nz = self._in_nz, self._out_nz
        self._in_nz = self._out_nz = 0
        if in_alive and not out_alive and in_n > 0:
            self.get_logger().warn(
                f'输入 {in_n} 帧/秒，但监控输出静默 → 判定卡死')
            if not self._dry:
                for name in MONITORS:
                    self._restart(name)
        elif (in_alive and out_alive and in_nz > 0 and out_nz == 0
              and self._blocked_pts < 3):
            # 2026-11-18（用户："不能自动恢复吗"）：监控"假活" ——
            # 状态是 active、输出帧率正常，但**每一帧都是 0**。
            # 原因之一是 TF 偶发外推失败（日志里的
            # "Lookup would require extrapolation into the past"），
            # 监控算不出前停区就按"看不见"处理 = 停车。
            # 原来看门狗只盯"输出静默"，这种"有输出但全零"完全漏掉，
            # 只能靠探索器取消目标 + 人工点"开始导航"。
            # 现在：Nav2 在要速度、监控却持续压零、且前停区里**没有真实障碍点**
            # → 判假活，自动重启监控。
            self._zero_run += 1.0
            if self._zero_run >= 2.0:
                self._zero_run = 0.0
                self.get_logger().warn(
                    f'Nav2 要速度（{in_nz} 帧/秒非零），监控却持续压零 2 s，'
                    f'且前停区无真实障碍（{self._blocked_pts} 点）→ 判定假活，重启监控')
                if not self._dry:
                    for name in MONITORS:
                        self._restart(name)
        elif not in_alive and not out_alive:
            # 空闲时两者都没数据是正常的，不处理
            self._zero_run = 0.0
        else:
            self._zero_run = 0.0


def _acquire_lock():
    """单实例锁：多个看门狗同时重启监控会把监控本身搞死。"""
    import fcntl
    os.makedirs('/tmp/fwr_runs', exist_ok=True)
    f = open('/tmp/fwr_runs/chain_watchdog.lock', 'w')
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.close()
        return None
    f.write(str(os.getpid()))
    f.flush()
    return f


def main() -> int:
    lock = _acquire_lock()
    if lock is None:
        print('已有看门狗实例在运行 → 本实例退出')
        return 0
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--restart-scan', action='store_true',
                    help='雷达断流超过 scan_stale_s 时自动重启 sllidar_node')
    ap.add_argument('--dedupe-scan', action='store_true',
                    help='/scan 出现多个发布者时结束全部雷达进程（默认只告警）')
    args = ap.parse_args()
    rclpy.init()
    node = Watchdog(args.dry_run)
    from rclpy.parameter import Parameter
    node.set_parameters([
        Parameter('restart_scan', Parameter.Type.BOOL, bool(args.restart_scan)),
        Parameter('dedupe_scan', Parameter.Type.BOOL, bool(args.dedupe_scan)),
    ])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass          # 重启导航栈时被 SIGINT/SIGTERM 打断属于正常，不要报"崩溃"
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
