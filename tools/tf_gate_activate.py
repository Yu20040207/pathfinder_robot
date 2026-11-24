#!/usr/bin/env python3
"""TF 真的就绪后再激活碰撞监控（替代 launch 里写死的 40 秒延迟）。

为什么需要（2026-11-19 现场 + 2026-11-24 复盘）
----------------------------------------------
`collision_monitor` 只要在 `odom→base_footprint` 的 TF 缓冲还没攒够历史时被
激活，第一次查 `laser_front→base_footprint` 就会报

    getTransform: laser_front->base_footprint, requested 447.30
    但 buffer 最早 447.49（Lookup would require extrapolation into the past）

之后节点**是 active 但不发布** `/cmd_vel_auto_forward_filtered`（活性但静默），
车原地停 2 分 8 秒。为了绕开它，launch 里把 lifecycle manager 延迟了
40 秒（12 s → 25 s → 40 s，25 s 时实测仍只差 0.4 s）。

代价是每次重启导航都要白等 25 秒（Nav2 15 s 就绪，安全链 90 s 才就绪）。

现在的做法
----------
监控节点照旧立刻起来（`unconfigured`，不发任何指令，安全）；
本脚本**盯着 TF**：`laser_front→base_footprint` 能在 `now-1.0s` 查到、
并且连续 `--steady-s` 秒都能查到（= 缓冲至少攒了 1 秒历史），
再调 `configure` + `activate` 并校验真的进了 `active`。

失败不报警也不阻塞：最多等 `--timeout-s`（默认 90 s）就退出，
`chain_watchdog` 仍会在后面补激活（兜底，只是慢一点）。
"""

import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros


def _state(node, target, timeout=2.0):
    cli = node.create_client(GetState, f'/{target}/get_state')
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    try:
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        res = fut.result()
        return res.current_state.label if res else None
    except Exception:                                   # noqa: BLE001
        return None
    finally:
        node.destroy_client(cli)


def _transition(node, target, transition_id, timeout=8.0):
    cli = node.create_client(ChangeState, f'/{target}/change_state')
    if not cli.wait_for_service(timeout_sec=timeout):
        return False
    try:
        req = ChangeState.Request()
        req.transition.id = transition_id
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        res = fut.result()
        return bool(res and res.success)
    except Exception:                                   # noqa: BLE001
        return False
    finally:
        node.destroy_client(cli)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--node-name', default='collision_monitor_forward')
    ap.add_argument('--target-frame', default='base_footprint')
    ap.add_argument('--source-frame', default='laser_front')
    ap.add_argument('--history-s', type=float, default=1.0,
                    help='要能在 now-1.0s 查到（= 缓冲够深，监控查 now-0.2s 不会外推）')
    ap.add_argument('--steady-s', type=float, default=2.0)
    ap.add_argument('--timeout-s', type=float, default=90.0)
    args, _ = ap.parse_known_args(argv)

    rclpy.init()
    node = Node('tf_gate_activate')
    buf = tf2_ros.Buffer(cache_time=Duration(seconds=60.0))
    tf2_ros.TransformListener(buf, node)                 # noqa: F841
    log = node.get_logger()
    t0 = time.monotonic()

    # 1) 等 TF 有足够历史（连续 steady_s 秒都能查到 now-history_s）
    ok_since = None
    while time.monotonic() - t0 < args.timeout_s:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            buf.lookup_transform(
                args.target_frame, args.source_frame,
                node.get_clock().now() - Duration(seconds=args.history_s))
            if ok_since is None:
                ok_since = time.monotonic()
            elif time.monotonic() - ok_since >= args.steady_s:
                break
        except Exception:                               # noqa: BLE001
            ok_since = None
    else:
        log.error('TF %s→%s 在 %.0f s 内没有稳定可用（离 %.1fs），放弃激活，'
                  '交给 chain_watchdog 兜底'
                  % (args.source_frame, args.target_frame, args.timeout_s,
                     args.history_s))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    waited = time.monotonic() - t0
    log.info('TF %s→%s 已稳定（等了 %.1f s）→ 激活 %s'
             % (args.source_frame, args.target_frame, waited, args.node_name))

    # 2) 按当前状态补齐 configure/activate
    st = _state(node, args.node_name)
    if st == 'active':
        log.info('%s 已经是 active，无需操作（%.1f s）'
                 % (args.node_name, time.monotonic() - t0))
        node.destroy_node()
        rclpy.shutdown()
        return 0
    if st in (None, 'unconfigured', 'finalized'):
        _transition(node, args.node_name, Transition.TRANSITION_CONFIGURE)
        time.sleep(0.5)
    ok = _transition(node, args.node_name, Transition.TRANSITION_ACTIVATE)
    if not ok:
        # 常见原因：刚 respawn、状态还没同步；等一下再试一次
        time.sleep(1.0)
        ok = _transition(node, args.node_name, Transition.TRANSITION_ACTIVATE)
    st = _state(node, args.node_name)
    if st == 'active' or ok:
        log.info('%s 激活完成：%s（总耗时 %.1f s）'
                 % (args.node_name, st or 'unknown', time.monotonic() - t0))
        rc = 0 if st == 'active' else 1
    else:
        log.error('%s 激活失败（状态 %s），交给 chain_watchdog 兜底'
                  % (args.node_name, st or 'unknown'))
        rc = 1
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
