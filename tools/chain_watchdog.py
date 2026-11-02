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
import sys
import time

import rclpy
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

    def _on_in(self, _msg):
        self._in += 1
        self._last_in = time.monotonic()

    def _on_out(self, _msg):
        self._out += 1
        self._last_out = time.monotonic()

    def _state_of(self, name):
        cli = self._state_clients.get(name)
        if cli is None or not cli.service_is_ready():
            return 'unknown'
        future = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        res = future.result()
        return res.current_state.label if res is not None else 'unknown'

    def _do(self, name, transition_id, label):
        client = self._change_clients[name]
        if not client.service_is_ready():
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        res = future.result()
        ok = res is not None and res.success
        self.get_logger().info('%s %s %s' % (name, label, '完成' if ok else '失败'))
        return ok

    def _restart(self, name):
        """按当前状态恢复。2026-10-31：新增 unconfigured 处理。

        原实现只做 deactivate->activate；当节点处于 unconfigured（lifecycle
        manager 掉线、bond 断裂）时两个转换都失败，整条自动链路永久断开 —
        现场表现是"规划有、车不动、目标全失败"。
        """
        if name not in self._change_clients:
            return
        state = self._state_of(name)
        if state == 'unconfigured':
            self._do(name, Transition.TRANSITION_CONFIGURE, 'configure')
            self._do(name, Transition.TRANSITION_ACTIVATE, 'activate')
        elif state == 'inactive':
            self._do(name, Transition.TRANSITION_ACTIVATE, 'activate')
        elif state == 'active':
            self._do(name, Transition.TRANSITION_DEACTIVATE, 'deactivate')
            self._do(name, Transition.TRANSITION_ACTIVATE, 'activate')
        else:
            # unknown（get_state 没答上来）时不能只试 activate —— 节点很可能已经
            # 是 active 但卡住，ACTIVATE 转换会报 "No transition matching 3"。
            # 这时走已知有效的 deactivate->activate 周期。
            self.get_logger().warn('%s 状态 %s，执行 deactivate->activate 周期'
                                   % (name, state))
            self._do(name, Transition.TRANSITION_DEACTIVATE, 'deactivate')
            self._do(name, Transition.TRANSITION_ACTIVATE, 'activate')

    def _scan_publishers(self):
        try:
            infos = self.get_publishers_info_by_topic('/scan')
            return [i.node_name for i in infos]
        except Exception:                          # noqa: BLE001
            return []

    def _on_scan(self, _msg) -> None:
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
        in_alive = now - self._last_in < 1.0
        out_alive = now - self._last_out < 1.0
        in_n, out_n = self._in, self._out
        self._in = self._out = 0
        if in_alive and not out_alive and in_n > 0:
            self.get_logger().warn(
                f'输入 {in_n} 帧/秒，但监控输出静默 → 判定卡死')
            if not self._dry:
                for name in MONITORS:
                    self._restart(name)
        elif not in_alive and not out_alive:
            # 空闲时两者都没数据是正常的，不处理
            pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    rclpy.init()
    node = Watchdog(args.dry_run)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
