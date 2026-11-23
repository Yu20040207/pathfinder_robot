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
from lifecycle_msgs.msg import TransitionEvent
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
        self._grace_s = float(os.environ.get('FWR_WATCHDOG_GRACE_S', '70'))
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
        self._last_out_zero = True
        self._zero_run = 0.0
        # 2026-11-19：冷却 + "输入持续流动"计数，修掉空闲误判。
        # 原来 `in_n > 0` 就判故障，空闲时 Nav2 偶尔发 1 帧零速也会被当成卡死，
        # 一天误触发 172 次 deactivate/activate，反而把状态机弄坏。
        self._in_run = 0.0
        # 2026-11-20：区分"导航中"和"空闲"。导航中 Nav2 会持续发**非零**速度，
        # 这时监控只要 4 s 不输出就是真卡死，不能再等 10 s（用户：走一下就停太久）。
        # 空闲时 Nav2 只发零速帧，仍用 out_silent_s=10 s，避免误杀。
        self._last_in_nz = 0.0
        self._fast_silent_s = float(
            os.environ.get('FWR_WATCHDOG_FAST_SILENT_S', '4'))
        self._ever_out = False       # 启动后是否发布过（区分"还没起来"）
        self._startup_fix_n = 0      # 启动期从未发布过时的重试次数          # 输入连续有数据的秒数
        self._last_fix = {}         # 每个监控上次动手的时间
        self._last_stack_restart = 0.0
        self._fix_cooldown_s = float(
            os.environ.get('FWR_WATCHDOG_FIX_COOLDOWN_S', '30'))
        self._stack_cooldown_s = float(
            os.environ.get('FWR_WATCHDOG_STACK_COOLDOWN_S', '300'))
        # 2026-11-19：三级（重启整个导航栈）**默认关闭**。
        # 现场实测：栈刚起来时监控会因 TF 未就绪而静默，三级重启栈会形成
        # "重启→再静默→再重启"的死循环（10:29 / 10:31 / 10:35 连续三次）。
        # 真正修法是 launch 里把碰撞监控的激活延后 12 s；三级改成显式开启。
        self._allow_stack_restart = os.environ.get(
            'FWR_WATCHDOG_ALLOW_STACK_RESTART', '0') == '1'
        # 2026-11-19：监控"还没激活"时本来就**不该有输出**。
        # 原来看门狗只看"输入有、输出无"就判卡死，于是在 launch 设的 40 s
        # 延迟激活窗口里把还没 active 的监控 deactivate 掉 →
        # lifecycle_manager 判 bond 丢失 → CRITICAL FAILURE → 重置 → 越修越坏
        # （实测 17:10:28 一次，之前 16:58:59 一次）。
        # 现在跟 lifecycle 的 transition_event，只有 state==active 才判故障。
        self._cm_state = {m: None for m in MONITORS}
        self._ever_active = {m: False for m in MONITORS}
        # 2026-11-20：初值设为启动时刻。原来初值 0.0 时 `now - 0` 是几十万秒，
        # 激活后的盲等窗口形同不存在 → 栈刚起来监控还没输出就被判卡死杀掉。
        self._cm_active_at = time.monotonic()   # 监控最近一次进入 active 的时刻
        for _m in MONITORS:
            self.create_subscription(
                TransitionEvent, f'/{_m}/transition_event',
                (lambda msg, name=_m: self._on_transition(name, msg)), 10)
        self._state_seed_at = 0.0
        self._state_inflight = set()
        # 2026-11-23：每次状态结果的时间戳。_health 里原来是"发起异步查询后
        # 立刻读 _cm_state"，读到的是**上一次**的旧值 —— 节点刚 respawn 成
        # active 时旧值还是 inactive → 误判 → 杀进程 → 再误判（死循环）。
        self._state_at = {}
        # 2026-11-23：熔断。5 分钟内杀到 4 次就停手 —— 说明"杀进程"这条路
        # 已经治不好它了（多半是 TF/雷达的根因没解决），继续杀只会让手机端
        # 一直"安全链等待中"。宁可放它静默，也不要制造 30 s 一轮的抖动。
        self._kill_times = []      # 正在异步查状态的监控
        self._last_cli_seed = 0.0         # 上次用 CLI 兜底的时间（必须有节流）
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
        self.declare_parameter('out_silent_s', 10.0)
        # 2026-11-22：监控"一直不是 active"也要管。见 _watch_inactive()。
        self.declare_parameter('inactive_grace_s', 90.0)
        self._inactive_since = {}
        self._inactive_retry = {}
        self.create_timer(5.0, self._health)
        self.get_logger().info(
            f'看门狗启动（{"只报告" if dry_run else "自动重启"}）：{IN_TOPIC} → {OUT_TOPIC}')

    def _on_transition(self, name, msg):
        try:
            prev = self._cm_state[name]
            self._cm_state[name] = msg.goal_state.label
            if msg.goal_state.label == 'active':
                self._ever_active[name] = True
                if prev != 'active':
                    self._cm_active_at = time.monotonic()
        except Exception:                        # noqa: BLE001
            pass

    def _seed_state(self):
        """看门狗启动时监控可能已经 active、收不到事件 → 主动查一次。

        2026-11-21：**不要用 `ros2 lifecycle get`**。实测每次 CLI 都要 ~2 s CPU
        （新起一个 DDS participant），而它在这台机器上还会超时
        （collision_monitor 日志里每 3 分钟一条
        `failed to send response to /get_state (timeout)`），于是状态永远是 unknown，
        看门狗每 15 s 再起两个 CLI → CPU 被吃满（负载一度 47），摇杆就"一卡一卡"。
        现在：优先用节点里已有的 GetState 客户端异步查（毫秒级、无子进程）；
        只有服务一直不就绪才退到 CLI，且 **至少间隔 120 s**。
        """
        polled = False
        for m in MONITORS:
            if self._cm_state.get(m) is None:
                polled = self._poll_state(m) or polled
        if polled:
            return
        now = time.monotonic()
        if now - self._last_cli_seed < 120.0:
            return
        self._last_cli_seed = now
        for m in MONITORS:
            if self._cm_state.get(m) is None:
                st = self._state_of(m)          # 兜底，节流 120 s
                if st != 'unknown':
                    self._cm_state[m] = st
                    if st == 'active':
                        self._ever_active[m] = True

    def _poll_state(self, name: str) -> bool:
        """用 lifecycle 服务异步查状态；返回 False 表示服务还没就绪。"""
        client = self._state_clients.get(name)
        if client is None or not client.service_is_ready():
            return False
        if name in self._state_inflight:
            return True
        self._state_inflight.add(name)
        try:
            fut = client.call_async(GetState.Request())
            fut.add_done_callback(
                lambda f, n=name: self._on_state_result(n, f))
        except Exception:                        # noqa: BLE001
            self._state_inflight.discard(name)
            return False
        return True

    def _on_state_result(self, name: str, fut) -> None:
        self._state_inflight.discard(name)
        try:
            res = fut.result()
        except Exception:                        # noqa: BLE001
            return
        label = getattr(getattr(res, 'current_state', None), 'label', None)
        if not label:
            return
        if label == 'active':
            if self._cm_state.get(name) != 'active':
                self._cm_active_at = time.monotonic()
            self._ever_active[name] = True
        self._cm_state[name] = label
        self._state_at[name] = time.monotonic()

    def _on_in(self, msg):
        self._in += 1
        self._last_in = time.monotonic()
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self._in_nz += 1
            self._last_in_nz = time.monotonic()

    def _on_out(self, msg):
        self._out += 1
        self._last_out = time.monotonic()
        self._ever_out = True
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self._out_nz += 1
            self._last_out_zero = False
        else:
            self._last_out_zero = True

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

    def _kill_safety_processes(self):
        """SIGKILL 碰撞监控 + 它的 lifecycle_manager，让 launch 的 respawn 重拉。

        2026-11-19：卡死时监控的执行器不再处理 lifecycle 服务，走服务一定失败；
        只有换一个进程才恢复。SIGKILL 是必须的（SIGTERM 实测会被忽略，
        结果新旧 manager 抢同一个监控）。
        """
        now = time.monotonic()
        self._kill_times = [t for t in self._kill_times if now - t < 300.0]
        if len(self._kill_times) >= 4:
            self.get_logger().error(
                '5 分钟内已重启安全进程 4 次 → 熔断，停止自动重启。'
                '请人工检查（ros2 lifecycle get /collision_monitor_forward、'
                '/scan 频率、/tf 是否齐全）')
            return
        self._kill_times.append(now)
        subprocess.run(['pkill', '-9', '-f',
                        'nav2_collision_monito[r]/collision_monitor'],
                       timeout=5, check=False)
        subprocess.run(['pkill', '-9', '-f',
                        '__node:=lifecycle_manager_collision'],
                       timeout=5, check=False)
        # 进程换了，缓存的旧状态一律作废（否则下一轮 _health 又拿旧值判死）
        for m in MONITORS:
            self._cm_state[m] = None
        self._state_at.clear()
        self._state_inflight.clear()
        self._cm_active_at = time.monotonic()

    def _revive(self, name, gap: float = 12.0):
        """按当前生命周期状态补齐 configure/activate。

        2026-11-19：杀进程后 launch respawn 出来的节点是 unconfigured，
        直接 activate 会报 "Transition is not registered"，必须先 configure；
        而 configure 之后立刻 activate 又会因为 tf 缓冲还没有历史而
        第一次查变换就失败（外推错误），所以两步之间要留 3 秒。
        """
        st = self._state_of(name)
        if st in ('unconfigured', 'unknown', 'finalized'):
            self._cli(name, 'configure')
            time.sleep(gap)
            st = self._state_of(name)
        if st == 'inactive':
            self._cli(name, 'activate')
            time.sleep(max(gap, 8.0))     # 激活后给监控 8 秒转起来再判定
        elif st == 'active':
            # 2026-11-23：**已经是 active 时不能立刻用 2 s 输出窗口判死**。
            # 刚被 respawn 的节点要几秒才重新发布，原来 _out_fresh() 立刻返回
            # False → 判"二级恢复失败" → 三级再杀一次（死亡螺旋的一环）。
            time.sleep(max(gap, 8.0))
            st = self._state_of(name)
        return self._out_fresh()

    def _restart(self, name):
        """不看状态，直接"停->起"，然后用输出是否恢复来判定。

        2026-11-18：这个看门狗原来先查状态再决定怎么转，但 rclpy 和 CLI 两种
        查法在现场都会返回 'unknown'（ros2 lifecycle get 10 s 超时），于是
        36 次重启全部失败、还要反复打扰监控。现在干脆不查 —— deactivate 失败
        也无所谓（可能已经 inactive），关键是后面那次 activate，然后用
        "输出是否恢复"验证效果。
        """
        self.get_logger().warn(f'{name} → deactivate/activate')
        self._cli(name, 'deactivate')
        time.sleep(1.5)
        ok = self._cli(name, 'activate')
        time.sleep(1.5)
        back = self._out_fresh()
        self.get_logger().info(
            f'{name} activate={"成功" if ok else "失败"}，输出'
            f'{"已恢复" if back else "仍静默"}')
        return back

    def _out_fresh(self, window: float = 2.0) -> bool:
        """监控输出是否新鲜（用它验证恢复是否真的生效）。"""
        return bool(self._last_out) and (time.monotonic() - self._last_out) < window

    def _watch_inactive(self, now: float) -> None:
        """监控长时间不是 active 就补齐 configure/activate。

        为什么需要（2026-11-22 现场）：
          * 启动期 TF 没就绪（雷达刚重启）会让 collision_monitor 首次 activate
            失败，lifecycle_manager 直接放弃 → 节点永远停在 inactive；
          * 旧逻辑只在"输入有帧、输出静默"时才恢复，而"不是 active 时没有输出
            是正常的"这条早退把它放过了；`not any(_ever_active)` 又放过一次；
          * 结果：手机端一直「等待安全链（碰撞监控）就绪…」，直到人工 activate。

        这里只看状态本身：非 active 持续 > inactive_grace_s（默认 90 s，
        足够覆盖启动 50 s）就 `_revive()` 一次；同一节点 30 s 冷却、最多 3 次，
        避免把 lifecycle 状态机刷坏。**只补激活，不做任何运动指令。**
        """
        if self._dry:
            return
        grace = float(self.get_parameter('inactive_grace_s').value)
        for m in MONITORS:
            st = self._cm_state.get(m)
            if st == 'active':
                self._inactive_since.pop(m, None)
                self._inactive_retry.pop(m, None)
                continue
            if st is None and m not in self._inactive_since:
                # 状态还没查到：交给 _seed_state，这里不猜
                continue
            since = self._inactive_since.setdefault(m, now)
            if now - since < grace:
                continue
            if now - self._last_fix.get(m, 0.0) < self._fix_cooldown_s:
                continue
            n = self._inactive_retry.get(m, 0) + 1
            self._inactive_retry[m] = n
            self._inactive_since[m] = now
            self._last_fix[m] = now
            self.get_logger().warn(
                f'{m} 已是 {st or "unknown"} 超过 {grace:.0f}s（第 {n}/3 次）'
                f' → 补 configure/activate（只补激活，不动车）')
            if self._revive(m, gap=3.0):
                self.get_logger().info(f'{m} 补激活成功，安全链应已恢复')
                self._inactive_retry.pop(m, None)
                self._inactive_since.pop(m, None)
            elif n >= 3:
                self.get_logger().error(
                    f'{m} 连续 {n} 次补激活失败 → 请人工检查'
                    f'（ros2 lifecycle set /{m} configure/activate）')

    def _fix(self, name: str, reason: str) -> None:
        """分级恢复：冷却 → deactivate/activate → cleanup/configure/activate
        → 自动重启导航栈。

        2026-11-19（用户要求 A+B）：
          * **冷却 30 s**：同一节点 30 s 内只动一次。原先空闲误判会连续
            deactivate/activate，把生命周期状态机打断（日志里的
            "timer is canceled" / "get_available_transitions timeout"）。
          * **升级链**：一级无效就走二级（节点级 cleanup→configure→activate，
            不重启整个栈）；二级仍无效才自动重启 fwr-nav（5 分钟冷却）。
            以前只打印"需要 bash /tmp/restart_nav.sh"，现场没人点就一直停着。
        """
        now = time.monotonic()
        last = self._last_fix.get(name, 0.0)
        if now - last < self._fix_cooldown_s:
            self.get_logger().info(
                f'{name} {reason}：冷却中（{now - last:.0f}s < '
                f'{self._fix_cooldown_s:.0f}s），跳过')
            return
        self._last_fix[name] = now

        # 2026-11-19：启动后**一次都没发布过**，说明是启动期 TF 还没就绪
        # （日志实证：`getTransform: laser_front->base_footprint，
        #   requested 447.30 但 buffer 最早 447.49`），节点被卡在首次变换查询。
        # 这时不能升级到二级/三级 —— 10:31 那次二级 cleanup + 三级重启栈，
        # 反而把 lifecycle_manager 的 bond 弄丢（"SERVER IS DOWN"）雪上加霜。
        # 只做"再激活一次"（最多 3 次，间隔由 30 s 冷却保证），不重启栈。
        if not self._ever_out:
            self._startup_fix_n += 1
            if self._startup_fix_n > 3:
                self.get_logger().error(
                    f'{name} 启动后始终无输出（TF 未就绪？），已重试 3 次，'
                    f'请重启导航栈或检查 /tf')
                return
            # 2026-11-19：启动期静默是"节点卡在首次 TF 查询"，
            # 它的执行器已经不处理 lifecycle 服务了 —— deactivate/activate
            # 只会把 lifecycle_manager 的 bond 弄丢（现场 16:58:59 报
            # "CRITICAL FAILURE: SERVER collision_monitor_forward IS DOWN"），
            # 之后节点被 reset 到 unconfigured，restart 的 activate 必然失败，
            # 车就停在那里直到进程被重拉（实测停了 2 分 8 秒）。
            # 现在直接走"杀进程让 launch respawn"，几秒就能恢复。
            self.get_logger().warn(
                f'{name} {reason}：启动后无输出，第 {self._startup_fix_n}/3 次'
                f'→ 直接重启监控进程（不做 deactivate，避免丢 bond）')
            self._kill_safety_processes()
            time.sleep(6.0)
            if self._out_fresh():
                self.get_logger().info(f'{name} 启动期恢复成功（进程已 respawn）')
                self._startup_fix_n = 0
            return

        # 2026-11-19：卡死时**别再 deactivate**。
        # 实测 deactivate 会让 lifecycle_manager 判 bond 丢失 →
        # CRITICAL FAILURE → 把监控 reset 成 unconfigured →
        # 之后的 activate 全部失败，车一直停（17:11/17:13/17:16 三次复现）。
        # 直接杀进程让 launch respawn 反而 8 秒内就恢复（17:16:26 实测）。
        self.get_logger().warn(
            f'{name} {reason} → 直接重启监控进程（不做 deactivate，避免丢 bond）')
        self._kill_safety_processes()
        time.sleep(10.0)
        if self._revive(name):
            self.get_logger().info(f'{name} 恢复成功（进程已 respawn）')
            return

        self.get_logger().warn(
            f'{name} 一级无效 → 二级恢复（按状态 configure/activate）')
        if self._revive(name) and self._out_fresh():
            self.get_logger().info(f'{name} 二级恢复成功（输出已恢复）')
            return

        # 三级：重启"安全子系统"进程（碰撞监控 + 它的 lifecycle_manager）。
        # 2026-11-19：监控卡死时它的执行器已经不再处理 lifecycle 服务，
        # 所以一级/二级（走服务）必然失败；实测只有"换一个进程"才恢复。
        # launch 里给这两个节点都开了 respawn，杀掉后约 3+2 s 自动拉起，
        # 比整栈重启（40~60 s）快一个数量级，也不会中断导航/定位。
        self.get_logger().warn(
            f'{name} 二级恢复失败 → 三级：杀掉监控进程让 launch respawn')
        self._kill_safety_processes()
        time.sleep(10.0)
        if self._revive(name):
            self.get_logger().info(f'{name} 三级恢复成功')
            return
        state = self._state_of(name)
        if state not in ('active',):
            self.get_logger().warn(
                f'{name} respawn 后状态 {state} → 手动 configure/activate')
            self._cli(name, 'configure')
            time.sleep(1.5)
            self._cli(name, 'activate')
        time.sleep(6.0)
        if self._out_fresh(4.0):
            self.get_logger().info(f'{name} 三级恢复成功（进程已 respawn）')
            self._startup_fix_n = 0
            return
        self.get_logger().error(f'{name} 三级（重启进程）也失败')

        if not self._allow_stack_restart:
            self.get_logger().error(
                f'{name} 需要人工重启导航栈'
                f'（bash /tmp/restart_nav.sh；自动整栈重启默认关闭）')
            return
        if now - self._last_stack_restart < self._stack_cooldown_s:
            self.get_logger().error(
                f'{name} 二级仍失败；导航栈 {now - self._last_stack_restart:.0f}s '
                f'前刚重启过（冷却 {self._stack_cooldown_s:.0f}s），暂不重启')
            return
        self._last_stack_restart = now
        self.get_logger().error(
            f'{name} 二级恢复失败 → 三级：自动重启导航栈（约 40~60 秒）')
        try:
            subprocess.run(['sudo', '-n', 'systemctl', 'restart', 'fwr-nav.service'],
                           timeout=180, check=False)
        except Exception as exc:                    # noqa: BLE001
            self.get_logger().error(f'重启导航栈失败: {exc}')

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
        # 2026-11-19：_health 原来不受启动宽限期约束，栈刚起来监控还没
        # active 就判"指令流动但输出静默"，一路升级到重启整个导航栈 ——
        # 复现了 10:29 那次自己把自己重启掉的事故。这里补上宽限期。
        if now - self._started < self._grace_s:
            return
        cmd_flowing = (now - self._last_in) < 1.0 and self._in_run >= 2.0
        # 2026-11-20：和 _check 保持一致 —— 导航中（有非零指令）用 4 s 阈值
        nav_active = (now - self._last_in_nz) < 2.0
        silent_need = self._fast_silent_s if nav_active else float(
            self.get_parameter('out_silent_s').value)
        out_silent = (self._ever_out and (now - self._last_out) > silent_need
                      and not (self._blocked_pts >= 3 and self._last_out_zero))
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

        # 2026-11-19：`ros2 lifecycle get` 在负载高时能卡 10 s，原来每 5 s 查一次
        # 两个监控，等于把执行器占满。现在只有"输出静默"或每 30 s 才查状态。
        query_state = out_silent or (
            now - getattr(self, '_last_state_check', 0.0) > 30.0)
        if query_state:
            self._last_state_check = now
        for name in MONITORS:
            if not query_state:
                if cmd_flowing and out_silent:
                    self._fix(name, '指令流动但输出静默')
                continue
            self._poll_state(name)              # 异步，毫秒级
            state = self._cm_state.get(name)
            # 2026-11-23：**只在状态是新鲜的时候才下判断**。
            # _poll_state 是异步的，回调要等几毫秒~几秒；原来这一行紧接着就读
            # _cm_state，读到的是上一轮的旧值。节点被 respawn 后旧值仍是
            # 'inactive'，于是刚恢复的监控被判"无效"→ 杀掉 → 再判 → 再杀，
            # 2026-11-23 实测一天 12 次三级重启、14 次进程重拉，
            # 手机端就表现为"安全链老是等待中"。
            fresh = (now - self._state_at.get(name, 0.0)) < 10.0
            if state == 'active':
                self._ever_active[name] = True
            if (state not in ('active', 'unknown')
                    and self._ever_active.get(name) and fresh):
                self.get_logger().warn('%s 处于 %s，自动恢复' % (name, state))
                self._fix(name, f'状态 {state}')
            elif state not in ('active', 'unknown'):
                # 启动期还没激活过：inactive 是正常状态，别去 deactivate
                pass
            elif cmd_flowing and out_silent:
                self.get_logger().warn(
                    '%s 指令在流动但输出静默 %.1fs，重启'
                    % (name, now - self._last_out))
                self._fix(name, '指令流动但输出静默')

    def _check(self) -> None:
        now = time.monotonic()
        # 2026-11-18：启动宽限期。导航栈刚起来时监控还没发布，
        # 看门狗会立刻判"卡死"并 deactivate/activate —— 而这恰恰是
        # collision_monitor "configure 超时/卡在 inactive" 的根源。
        if now - self._started < self._grace_s:
            self._in = self._out = 0
            return
        # 2026-11-22：**先补激活，再谈"静默是否正常"**。
        # 用户报"导航模式一直显示等待安全链就绪"。实测 collision_monitor_forward
        # 停在 inactive [2] 40 分钟没人管：08:41 后雷达反复超时重启，
        # 监控首次激活时 TF 还没就绪（getTransform 外推失败）→
        # lifecycle_manager 放弃激活 → 节点永远 inactive。
        # 而下面两个早退分支（"还没见过 active"、"不是 active 时静默正常"）
        # 都会直接 return，谁也不去补 activate → 过滤后速度恒无 → chain=false。
        self._watch_inactive(now)
        # 状态未知就每 15 秒补查一次（放到线程里，避免 CLI 卡住定时器）
        if any(v is None for v in self._cm_state.values()) and \
                now - self._state_seed_at > 15.0:
            self._state_seed_at = now
            import threading
            threading.Thread(target=self._seed_state, daemon=True).start()
        # 本次运行还一次没见过监控 active（刚启动/正在配置）→ 静默是正常的
        if not any(self._ever_active.values()):
            self._in = self._out = 0
            self._in_run = 0.0
            return
        # 监控不是 active（还没激活 / 正在 reset）时，没有输出是**正常**的
        states = [self._cm_state.get(m) for m in MONITORS]
        if all(st is not None for st in states) and 'active' not in states:
            self._in = self._out = 0
            return
        in_alive = now - self._last_in < 1.0
        out_alive = now - self._last_out < 1.0
        in_n, out_n = self._in, self._out
        self._in = self._out = 0
        in_nz, out_nz = self._in_nz, self._out_nz
        self._in_nz = self._out_nz = 0
        # 输入连续流动计时（空闲时 Nav2 偶尔发 1 帧零速，不能算"在动"）
        if in_alive and in_n >= 5:
            self._in_run += 1.0
        else:
            self._in_run = 0.0
        # 2026-11-19：2 s 太敏感 —— 实测只是"指令刚发过来、监控还没转起来"
        # 就判卡死并 deactivate，反而把 bond 弄丢（17:13:38 现场）。
        # 20 Hz 的节点 10 s 一帧都没有才是真坏。
        # 2026-11-20：导航中（最近 2 s 有非零指令）= 用户正看着车走，
        # 静默 4 s 就修；空闲/刚启动仍用 10 s，避免误杀。
        nav_active = (now - self._last_in_nz) < 2.0
        amplify = 8.0 if nav_active else 20.0
        if now - self._cm_active_at < amplify:
            self._in_run = 0.0
            if in_alive and not out_alive:
                return
        silent_need = self._fast_silent_s if nav_active else float(
            self.get_parameter('out_silent_s').value)
        # 2026-11-20：前方**真有障碍**、监控最后一次输出是零速时，静默是
        # stop_pub_timeout 的正常行为（Nav2 停止重复发零），不是卡死。
        # 之前把这种情况也判成卡死 → 车停在墙前时看门狗每 35 s 杀一次监控，
        # 速度链断断续续（15:31~15:35 连续 4 轮）。
        legit_stop = (self._blocked_pts >= 3 and self._last_out_zero)
        if legit_stop:
            self._in_run = 0.0
            if in_alive and not out_alive:
                return
        if in_alive and not out_alive and in_n >= 5 and self._in_run >= silent_need:
            self.get_logger().warn(
                f'输入 {in_n} 帧/秒（连续 {self._in_run:.0f}s，'
                f'{"导航中" if nav_active else "空闲"}阈值 {silent_need:.0f}s），'
                f'监控输出静默 → 判定卡死')
            if not self._dry:
                for name in MONITORS:
                    self._fix(name, '输出静默')
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
                        self._fix(name, '持续压零')
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
    except Exception:                           # noqa: BLE001
        # 2026-11-22：以前任何未捕获异常都会静默退出（launch 没有 respawn），
        # 之后安全链再也没人守 —— 日志里连一行都没有。现在打印再以非 0 退出。
        import traceback
        traceback.print_exc()
        node.get_logger().fatal('看门狗异常退出，交给 launch respawn')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
