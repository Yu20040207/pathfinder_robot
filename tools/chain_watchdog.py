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
MONITORS = ('collision_monitor_forward', 'collision_monitor_reverse')


class Watchdog(Node):
    def __init__(self, dry_run: bool):
        super().__init__('chain_watchdog')
        self._dry = dry_run
        self._in = 0
        self._out = 0
        self._last_in = 0.0
        self._last_out = 0.0
        self.create_subscription(Twist, IN_TOPIC, self._on_in, 20)
        self.create_subscription(Twist, OUT_TOPIC, self._on_out, 20)
        self._change_clients = {
            m: self.create_client(ChangeState, f'/{m}/change_state') for m in MONITORS
        }
        self._state_clients = {
            m: self.create_client(GetState, f'/{m}/get_state') for m in MONITORS
        }
        self.create_timer(1.0, self._check)
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
            self.get_logger().warn('%s 状态 %s，尝试直接激活' % (name, state))
            self._do(name, Transition.TRANSITION_ACTIVATE, 'activate')

    def _health(self):
        """周期健康检查：任何监控不是 active 就恢复。"""
        if self._dry:
            return
        for name in MONITORS:
            state = self._state_of(name)
            if state not in ('active', 'unknown'):
                self.get_logger().warn('%s 处于 %s，自动恢复' % (name, state))
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
