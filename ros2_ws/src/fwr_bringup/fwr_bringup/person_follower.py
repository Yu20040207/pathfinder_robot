"""跟随控制节点：跟人走 + 实时避障（走手动通道 `/cmd_vel_manual`）。

与第一版的区别（2026-11-21 用户反馈三件事）
----------------------------------------
1. **远近只看框的大小**（``box_h`` 占画面比例），不再用身高估米数。
   估米数在人走近时框被画面裁掉，距离卡在 ~2 m，车永远到不了"该停"的点。
2. **走廊避障**：只有横向 ±0.22 m（车身半宽）内的点才当障碍，
   门框在门两侧不会被判成障碍 → 能过门。
3. **转向更快**：``yaw_kp`` 2.2→3.5、``wmax`` 0.6→0.9。
   这些参数只属于跟随节点，不动导航的 `real_nav2.yaml`。

为什么走 `/cmd_vel_manual`
------------------------
底盘 `base_bridge` 只认 `/cmd_vel_auto_safe`（需 nav 栈）和 `/cmd_vel_manual`。
跟随是页面发起的手动行为，manual 通道在没起导航时也通，且天然受
`motion_enabled` + `manual_enabled` 双门控——**代理不用、也不能替用户开门**。

三层保护
--------
1. 本节点：雷达走廊 + 侧向净空 + 框大小
2. 底盘：motion_enabled / manual_enabled / 传感器就绪 / 指令 TTL
3. 看门狗：页面每 1 s 发心跳，2 s 收不到就停车
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from fwr_bringup.follow_core import (DEFAULTS, OscGuard, adopt_moving_track, bearing_ema, cam_bearing, decide, door_clear, door_metrics, door_swept_radius, flip_confirm, lidar_backed, lidar_drive_allowed, lidar_follow, lidar_search_cmd, limit_pair, passage_metrics, pick_lidar_track, predict_person, reacquire_track, reacq_cap, scan_metrics)


class PersonFollower(Node):
    def __init__(self) -> None:
        super().__init__('person_follower')
        p = self.declare_parameter
        # 20 Hz 与手动摇杆同频：10 Hz 更新对固件速度环来说太粗，车会一顿一顿
        self.follow_hz = float(p('follow_hz', 20.0).value)
        self.heartbeat_timeout = float(p('heartbeat_timeout', 3.0).value)
        # 框中心 EMA：0.35 → 0.5（2026-11-26，跟随转弯"跟不上"）。
        # α=0.35 时滞后 ≈(1-α)/α=1.86 帧，8 Hz 下约 230 ms；α=0.5 只剩 1 帧 ≈83 ms。
        # 抖动的兜底仍在 `OscGuard`（观测到 ω 反复变号会自动收增益），别把它关掉。
        self.smooth_alpha = float(p('smooth_alpha', 0.50).value)   # 框的 EMA
        # 重新认领后的"软对准"（2026-11-26 用户报：掉头重新看到人后左右摆）
        self.reacq_soft_s = float(p('reacq_soft_s', 0.80).value)
        self.reacq_wmax = float(p('reacq_wmax', 0.40).value)
        self.reacq_soft_ex = float(p('reacq_soft_ex', 0.45).value)
        # 跟随专用参数（ROS 参数可覆盖，默认值见 follow_core.DEFAULTS）
        self.p = {k: type(v)(p(k, v).value) for k, v in DEFAULTS.items()}
        self.v_accel = float(p('v_accel', 0.60).value)      # m/s²
        self.w_accel = float(p('w_accel', 3.00).value)      # rad/s²

        self.enabled = False
        self._heartbeat = 0.0
        self._zero_until = 0.0
        self._persons = {'sel': -1, 'tracking': False, 'lost_s': 0.0,
                         'persons': [], 'stamp': 0.0}
        self.clear_hard = math.inf
        self.clear_slow = math.inf
        self.left = self.right = math.inf
        self.rear_hard = math.inf
        self.rear_slow = math.inf
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self._ema = {'h': None, 'cx': None}   # 框高 / 框中心的平滑值
        self._last_person = None              # 上一帧匹配到的框（掉帧时沿用）
        self._last_seen = 0.0
        self._size_hold = False               # 因"太近"停过 → 等框缩到 resume_box_h
        self._back_hold = False               # 人贴脸 → 后退，退到 back_release 才停
        # 门洞（2026-11-21 第四版）：门框在车身走廊外，跟框转向会顶门框 →
        # 测到窄通道就改瞄门洞中心、进门前站正、门内低速直线走。
        self.gate_gap = math.inf
        self.gate_center = 0.0
        self._gate_hold = False
        self._gate_seen_at = 0.0
        self.gate_center_last = 0.0
        # PD 转向：框中心偏差的变化率（2026-11-21 晚修左右摆）
        self._prev_cx = None
        self._prev_cx_t = 0.0
        self.err_dot = 0.0
        self._seen_prev = False      # 上一帧有没有框（判"重新认领"）
        self._reacq_until = 0.0      # 软对准窗口截止时刻
        self._osc = OscGuard()      # 左右摆自动阻尼（见 follow_core.OscGuard）
        self.osc_gain = 1.0
        # 雷达方位滤波/滞回/换轨迹防抖（2026-11-26 用户报"雷达接管时左右摆很大"）
        self._lidar_b_s = None       # 平滑后的方位（rad）
        self._lidar_b_t = 0.0
        self._lidar_rate = 0.0       # 方位变化率（rad/s，闭合为负）
        self._lidar_face_hold = False
        self._lidar_last_ok = None   # (bearing, track_id) 上一次可信的方位
        self._lidar_flip = None      # (bearing, track_id, since) 突变候选
        self._lidar_mem = None       # 雷达自己记的位置（不污染相机真值 _person_mem）
        # 门内丢人 → 先出门再找人（2026-11-21 用户要求）
        self._last_side = 0.0        # 最后一次看到人时，人在画面 +1 右 / -1 左
        self._exit_gate = False      # 正在"顶着丢人把门洞走完"
        self._exit_elapsed = 0.0
        self._search_until = 0.0     # 出门后原地找人的截止时间
        # 整车出门判定（2026-11-21 用户："前雷达出去不等于车出去，我还有后雷达"）
        self.door_lon = None         # 门线相对车身的纵向位置（360° 合并扫描）
        self._door_at = 0.0          # 上一次真测到门线的时刻（丢门线时沿用）
        self.door_is_clear = True
        # "显示跟随中但车不动"检测（2026-11-21 实机踩坑）：底盘整帧丢弃命令时
        # 页面完全看不出来。这里对比"我发的 v"和"里程计实际速度"。
        self.odom_v = 0.0
        self._odom_at = 0.0
        self._nomotion_s = 0.0
        self.nomotion = False
        self._sel_prev = -1
        # —— P2 相机 + 雷达融合（2026-11-24）——
        # 相机看不到人时（人绕到车侧/车后），用刚才被相机点名的同一条雷达轨迹续跟。
        # ⚠️ 雷达**不许自己起跟**：没有过相机认领的轨迹一律不跟（家具/路人）。
        self.lidar_topic = str(p('lidar_topic', '/robot/lidar/persons').value)
        self._tracks = {}          # id -> {'x','y','r','bearing','at'}（车体系，已换算）
        self._tracks_at = 0.0      # 上一次收到雷达轨迹的时刻
        self.pose = (0.0, 0.0, 0.0)   # odom 位姿（x, y, yaw），换算轨迹到车体系要用
        self._pose_at = 0.0
        self._lidar_id = -1        # 相机点名过的那条雷达轨迹 id
        self._lidar_at = 0.0       # 上一次"相机与雷达对上"的时刻
        self._lidar_close = False  # 雷达接管时因"太近"停过 → 等退到 resume 距离
        self._lidar_bearing = None  # 刚才认领/续跟那条轨迹的方位、距离
        self._lidar_r = None        # （换号时按这对值重新认领）
        self._lidar_seen = 0.0      # 雷达**最后一次真正在用目标**的时刻。
        # 「人最后在哪」的 odom 系记忆：人绕到车后相机看不到时，靠它做匀速预测，
        # 轨迹换号也能按"应该出现的位置"认回来（2026-11-26 用户报"跟丢后走旁边也不认"）。
        self._person_mem = None
        self._blind_since = 0.0     # 相机最后一次看到人的时刻（0 = 现在看得到）
        self._lidar_static_since = 0.0   # 续跟目标"静止且无相机背书"的起点
        # ⚠️ 别拿 `_lidar_at` 当这个用：它只在"相机看到人并认领"时刷新，
        # 人一绕到车后就不动了 → 搜索窗口会在 2.5 s 后无故失效，
        # 而实际续跟窗口是 lidar_hold_s 8 s（2026-11-26 修）。
        self.src = 'none'          # cam / lidar / none（本帧目标来源）

        self.cmd = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.state = self.create_publisher(String, '/follow/state', 10)
        self.create_subscription(String, '/robot/vision/persons', self.on_persons, 10)
        self.create_subscription(String, self.lidar_topic, self.on_lidar, 10)
        self.create_subscription(String, '/follow/enable', self.on_enable, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(LaserScan, '/scan_merged', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / max(1.0, self.follow_hz), self.tick)
        self.get_logger().info(
            '跟随节点就绪：框高 %.0f%% 停（%.0f%% 起走）/ %.0f%% 后退，vmax %.2f m/s，'
            'wmax %.2f rad/s，走廊 ±%.2f m（车头 %.2f m 停 / 车尾 %.2f m 禁退），'
            '偏离中线 %.0f%% 原地转向，转向 PD kp %.1f/kd %.1f（跟随 ω≤%.2f）'
            % (self.p['stop_box_h'] * 100, self.p['resume_box_h'] * 100,
               self.p['back_box_h'] * 100, self.p['vmax'], self.p['wmax'],
               self.p['corridor_m'], self.p['front_hard_m'], self.p['back_hard_m'],
               self.p['face_ex'] * 100, self.p['yaw_kp'], self.p['yaw_kd'],
               self.p['steer_wmax']))

    # ------------------------------------------------------------ 回调
    def on_odom(self, msg: Odometry) -> None:
        self.odom_v = float(msg.twist.twist.linear.x)
        self._odom_at = time.monotonic()
        # P2：把雷达轨迹（odom 系）换算到车体系要用位姿；跟随只在车体系里算
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pos = msg.pose.pose.position
        self.pose = (float(pos.x), float(pos.y), yaw)
        self._pose_at = self._odom_at

    def on_lidar(self, msg: String) -> None:
        """收 `leg_tracker` 的轨迹（odom 系 JSON）→ 换算成车体系的方位/距离。

        只做换算和缓存，**不在这里做任何决策**（决策在 tick 里，20 Hz 和速度同频）。
        """
        try:
            data = json.loads(msg.data or '{}')
        except Exception:                        # noqa: BLE001
            return
        if (time.monotonic() - self._pose_at) > 1.0:
            return                               # 没位姿就不换算（宁可不跟）
        px, py, pyaw = self.pose
        now = time.monotonic()
        tracks = {}
        for tr in data.get('tracks', []):
            try:
                dx = float(tr['x']) - px
                dy = float(tr['y']) - py
                tid = int(tr['id'])
            except (KeyError, TypeError, ValueError):
                continue
            r = math.hypot(dx, dy)
            if not (0.3 <= r <= 6.0):
                continue
            # 车体系方位角：左正（REP-103），与 cam_bearing 符号一致
            bearing = math.atan2(dy, dx) - pyaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            # ⚠️ 'id' 必须放进条目里：pick_lidar_track() 直接读它
            # （2026-11-24 漏了 → 第一次认领就 KeyError，节点整个退出）
            tracks[tid] = {'id': tid, 'x': dx, 'y': dy, 'r': r,
                           'bearing': bearing, 'at': now,
                           'static': bool(tr.get('static')),
                           # 相机背书 + 速度：pick/reacquire 用它排除"动的家具"
                           'cam': bool(tr.get('cam')),
                           # odom 坐标（预测/按位置认领用；`x/y` 是车体系的 dx/dy）
                           'ox': float(tr['x']), 'oy': float(tr['y']),
                           'vx': float(tr.get('vx') or 0.0),
                           'vy': float(tr.get('vy') or 0.0),
                           'v': float(math.hypot(float(tr.get('vx') or 0.0),
                                                 float(tr.get('vy') or 0.0)))}
        self._tracks = tracks
        self._tracks_at = now


    def on_enable(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or '{}')
        except Exception:                        # noqa: BLE001
            return
        want = bool(data.get('on'))
        self._heartbeat = time.monotonic()
        if want and not self.enabled:
            self.get_logger().info('开始跟随')
        elif not want and self.enabled:
            self.get_logger().info('取消跟随 → 停车')
            # ⚠️ 只补发 0.3 s 的零速度：页面可能马上在用摇杆接管，
            # 补发 2 s（20 帧）会跟摇杆抢同一个 /cmd_vel_manual → 一卡一卡（2026-11-21）。
            self._zero_until = time.monotonic() + 0.3
        self.enabled = want
        if not want:
            self.v_cmd = self.w_cmd = 0.0
            self._publish(0.0, 0.0)

    def on_persons(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or '{}')
        except Exception:                        # noqa: BLE001
            return
        self._persons = {'sel': int(data.get('sel', -1)),
                         'tracking': bool(data.get('tracking')),
                         'lost_s': float(data.get('lost_s', 0.0)),
                         'persons': data.get('persons', []),
                         'stamp': time.monotonic()}

    def on_scan(self, msg: LaserScan) -> None:
        (self.clear_hard, self.clear_slow, self.left, self.right,
         self.rear_hard, self.rear_slow) = scan_metrics(
            msg.ranges, msg.angle_min, msg.angle_increment, self.p)
        gate = passage_metrics(msg.ranges, msg.angle_min, msg.angle_increment, self.p)
        if gate is None:
            self.gate_gap = math.inf
            # 迟滞：门洞消失后保持 gate_hold_s，避免在阈值上抖
            if self._gate_hold and (time.monotonic() - self._gate_seen_at
                                    > self.p['gate_hold_s']):
                self._gate_hold = False
        else:
            self.gate_gap, self.gate_center = gate
            self.gate_center_last = self.gate_center
            self._gate_hold = True
            self._gate_seen_at = time.monotonic()

        # 门线位置（前+后雷达，360°）：判断**整车**是否已经开出门框。
        # 只看前雷达会出现"车头刚过门线就以为出去了，其实车尾还在门里"。
        door = door_metrics(msg.ranges, msg.angle_min, msg.angle_increment, self.p)
        now_s = time.monotonic()
        if door is None:
            # ⚠️ 2026-11-22：门线**一时没测到**不能立刻当成"整车出去了"。
            # 遮挡、车身挡住后雷达、门线超出 ±1.3 m 视场都会让 door_metrics 返回 None，
            # 旧写法直接 door_is_clear=True → 车还在门框里就开始原地转身。
            # 现在沿用最近 door_lost_hold_s（0.6 s）内的测量；超时才算真出去。
            if (self.door_lon is not None and
                    now_s - self._door_at <= self.p['door_lost_hold_s']):
                self.door_is_clear = door_clear(self.door_lon, self.p)
            else:
                self.door_lon = None
                self.door_is_clear = True
        else:
            self.door_lon = door[0]
            self._door_at = now_s
            self.door_is_clear = door_clear(door[0], self.p)

    # ------------------------------------------------------------ 主循环
    def _publish(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd.publish(msg)

    def _lidar_bearing_filter(self, b_raw: float, now: float) -> float:
        """方位 EMA + 变化率（纯逻辑在 `follow_core.bearing_ema`）。"""
        dt = now - self._lidar_b_t
        smoothed, rate = bearing_ema(self._lidar_b_s, b_raw, dt,
                                    float(self.p['lidar_bearing_smooth']))
        if self._lidar_b_s is not None:
            self._lidar_rate += 0.5 * (rate - self._lidar_rate)
        else:
            self._lidar_rate = rate
        self._lidar_b_s = smoothed
        self._lidar_b_t = now
        return smoothed

    def _lidar_bearing_trusted(self, b: float, tid: int, now: float) -> bool:
        """换轨迹 + 方位突变要不要信（纯逻辑在 `follow_core.flip_confirm`）。"""
        ok, last_ok, flip = flip_confirm(
            self._lidar_last_ok, self._lidar_flip, b, tid, now,
            float(self.p['lidar_flip_reject_rad']),
            float(self.p['lidar_flip_confirm_s']))
        self._lidar_last_ok = last_ok
        self._lidar_flip = flip
        return ok

    def _state(self, reason: str, v: float, w: float, box_h: float) -> None:
        fin = lambda x: None if math.isinf(x) else round(x, 2)   # noqa: E731
        payload = {'on': self.enabled, 'v': round(v, 3), 'w': round(w, 3),
                   'reason': reason, 'box_h': round(box_h, 3),
                   'rear_m': round(self.rear_slow, 2) if math.isfinite(self.rear_slow) else 99.0,
                   'face_ex': self.p['face_ex'], 'back_box_h': self.p['back_box_h'],
                   'stop_box_h': self.p['stop_box_h'], 'go_box_h': self.p['go_box_h'],
                   'front_m': fin(self.clear_hard), 'side_l': fin(self.left),
                   'side_r': fin(self.right),
                   # 门洞（第四版）：页面/日志都用得上，判断"卡门框"时先看这两个数
                   'gate_on': bool(self._gate_hold), 'gate_gap': fin(self.gate_gap),
                   'gate_c': round(self.gate_center, 3),
                   'gate_exit': bool(self._exit_gate),
                   'door_lon': (round(self.door_lon, 2)
                                if self.door_lon is not None else None),
                   'door_clear': bool(self.door_is_clear),
                   'door_thresh': round(-(door_swept_radius(self.p)
                                          + self.p['door_clear_margin_m']), 3),
                   'mps': (round(self.odom_v, 3)
                           if (time.monotonic() - self._odom_at) < 1.0 else None),
                   'nomotion': bool(self.nomotion),
                   'err_dot': round(float(self.err_dot), 2),
                   'osc_gain': round(float(self.osc_gain), 2),
                   'lidar_bearing_deg': (round(math.degrees(self._lidar_bearing), 1)
                                         if self._lidar_bearing is not None else None),
                   'lidar_rate_deg_s': round(math.degrees(self._lidar_rate), 1),
                   'lidar_face': bool(self._lidar_face_hold),
                   'search_side': self._last_side,
                   'src': self.src, 'lidar_id': int(self._lidar_id),
                   'lidar_r': (round(self._tracks[self._lidar_id]['r'], 2)
                               if (self.src == 'lidar'
                                   and self._lidar_id in self._tracks) else None),
                   'lidar_age': (round(time.monotonic() - self._lidar_at, 2)
                                 if self._lidar_id >= 0 else None),
                   'sel': int(self._persons.get('sel', -1)),
                   'tracking': bool(self._persons.get('tracking', False)),
                   't': time.time()}
        self.state.publish(String(data=json.dumps(payload)))

    def tick(self) -> None:
        now = time.monotonic()
        dt = 1.0 / max(1.0, self.follow_hz)
        if self.enabled and (now - self._heartbeat) >= self.heartbeat_timeout:
            self.get_logger().warn('跟随心跳超时（页面没发 keepalive）→ 停车',
                                   throttle_duration_sec=5.0)
            self.enabled = False
            self._zero_until = now + 2.0
        if not self.enabled:
            if now < self._zero_until:      # 关闭后补发 2 s 零速度，确保停稳
                self._publish(0.0, 0.0)
            self._ema = {'h': None, 'cx': None}
            self._seen_prev = False
            self.err_dot = 0.0
            self._prev_cx = None
            self._state('off', 0.0, 0.0, 0.0)
            return

        data = self._persons
        sel = data.get('sel', -1)
        if sel != self._sel_prev:             # 换人 → 清掉平滑和历史框
            self._sel_prev = sel
            self._ema = {'h': None, 'cx': None}
            self._last_person = None
            self._seen_prev = False      # 换人也要走一次"重新认领"的软对准
            self._reacq_until = 0.0
            self._size_hold = False
            # 换人 = 换目标，雷达认领必须重来（否则会继续跟上一个被点名的人）
            self._lidar_id = -1
            self._lidar_bearing = None
            self._lidar_r = None
            self._lidar_seen = 0.0
            self._person_mem = None
            self._blind_since = 0.0
            self._lidar_static_since = 0.0
            self._lidar_close = False
        person = None
        for item in data.get('persons', []):
            # 2026-11-22：检测器在"短暂丢失"的 1 s 内会补发 stale 框（让页面绿框
            # 不变白）。控制侧**不吃**这种框 —— 继续走自己的 lost_stop_s 0.8 s
            # 沿用上一帧逻辑，别把"丢失"伪装成"看见"。
            if item.get('stale'):
                continue
            if item.get('id') == sel:
                person = item
                break
        lost_s = 0.0 if data.get('tracking') else float(data.get('lost_s', 0.0) or 0.0)

        if person is not None:
            self._last_person = dict(person)
            self._last_seen = now
            cx_now = float(person.get('x', 0.5)) + float(person.get('w', 0.0)) / 2.0
            if cx_now > 0.56:            # 人在画面右边（出门后要往右转找）
                self._last_side = 1.0
            elif cx_now < 0.44:
                self._last_side = -1.0
        elif (self._last_person is not None and sel >= 0
              and (now - self._last_seen) <= self.p['lost_stop_s']):
            # ⚠️ 检测器 8 Hz，YOLO 偶尔漏一帧。漏帧时**沿用上一帧的框**继续控制，
            # 否则每漏一帧就 v=0 急刹一次 → 用户看到的"一卡一卡"（2026-11-21）。
            person = dict(self._last_person)

        if person is not None:
            # ⚠️ 重新认领（掉头/遮挡后再次看见人）：**必须清掉旧的平滑状态**。
            # 2026-11-26 用户报"掉头重新识别到人之后左右摆"：旧 `_ema['cx']` 还是
            # 掉头前那一侧的中心，新框在另一侧 → 平滑值要爬好几帧才到，这期间
            # 误差方向是错的 → 车先往反方向转，再纠正 = 左摆右摆。
            if not self._seen_prev:
                self._ema['cx'] = None
                self._ema['h'] = None
                self.err_dot = 0.0
                self._prev_cx = None
                self._reacq_until = now + self.reacq_soft_s
            a = max(0.05, min(1.0, self.smooth_alpha))
            h_raw = float(person.get('h', 0.0))
            cx_raw = float(person['x']) + float(person['w']) / 2.0
            self._ema['h'] = h_raw if self._ema['h'] is None else (
                a * h_raw + (1 - a) * self._ema['h'])
            self._ema['cx'] = cx_raw if self._ema['cx'] is None else (
                a * cx_raw + (1 - a) * self._ema['cx'])
            person = dict(person)
            person['h'] = self._ema['h']
            person['x'] = self._ema['cx'] - float(person['w']) / 2.0

            # 平滑后的框中心变化率（给 PD 的阻尼项用）。
            # 注意必须用**平滑后**的 cx，原始框每帧抖 1~2% 会把 d 项放大成噪声。
            cx = self._ema['cx']
            t_now = time.monotonic()
            if self._prev_cx is None or (t_now - self._prev_cx_t) <= 1e-3:
                self.err_dot = 0.0
            else:
                raw = (cx - self._prev_cx) / (t_now - self._prev_cx_t)
                raw = max(-3.0, min(3.0, raw))
                self.err_dot += 0.5 * (raw - self.err_dot)
            self._prev_cx, self._prev_cx_t = cx, t_now
        else:
            self.err_dot = 0.0
            self._prev_cx = None

        box_h = float(person.get('h', 0.0)) if person else 0.0
        if person is not None:
            # 后退迟滞：贴脸(≥92%) 才开始退，退到 85% 以下才停
            if box_h >= self.p['back_box_h']:
                self._back_hold = True
            elif box_h <= self.p['back_release_box_h']:
                self._back_hold = False
            if self._back_hold:
                self._size_hold = False
            elif box_h >= self.p['stop_box_h']:
                self._size_hold = True
            elif box_h <= self.p['resume_box_h']:
                self._size_hold = False

        # —— 门内丢人状态机（2026-11-21 用户要求：先出门，再原地找人）——
        lost_now = (person is None) or (lost_s > self.p['lost_stop_s'])
        if self._exit_gate:
            self._exit_elapsed += dt
            if not lost_now:
                self._exit_gate = False                  # 人回来了 → 恢复正常跟随
            elif not self._gate_hold and self.door_is_clear:
                # **整车**都出了门框（门线退到车尾后面）→ 才允许原地转身找人。
                # 2026-11-21 用户：前雷达出去就转，会把车尾/车角刮在门框上。
                self._exit_gate = False
                self._search_until = now + self.p['search_turn_s']
                side_txt = ('右' if self._last_side > 0 else '左') \
                    if abs(self._last_side) > 0 else '人最后出现的一侧'
                self.get_logger().info(
                    '人先出门了：车已穿过门框，原地向%s转身找人' % side_txt,
                    throttle_duration_sec=5.0)
            elif self._exit_elapsed > self.p['gate_exit_max_s']:
                self._exit_gate = False                  # 出不去就别硬顶
        elif (lost_now and self._gate_hold and self.p['gate_exit_enabled']
              and abs(self._last_side) > 0):
            self._exit_gate = True
            self._exit_elapsed = 0.0
        search_now = (not self._exit_gate) and lost_now and now < self._search_until

        # —— P2 融合（2026-11-24）——
        # 相机看得见：用它的方位去认领一条雷达轨迹（黏住上次认领的 id）。
        # 相机看不见：只允许"刚才被相机点名的那一条"在 lidar_hold_s 内续跟。
        # ⚠️ 没有相机认领过的轨迹一律不跟 —— 家具/路人永远不会被跟。
        if person is not None:
            self._blind_since = 0.0
        elif self._blind_since == 0.0:
            self._blind_since = now
        self.src = 'cam' if person is not None else 'none'
        lidar_target = None
        if person is not None:
            cx_for_assoc = (self._ema['cx'] if self._ema['cx'] is not None
                            else float(person['x']) + float(person['w']) / 2.0)
            # 只拿"不像家具"的轨迹去认领（static 的是椅子/桌沿，相机看到的是人）
            picked = pick_lidar_track(
                [t for t in self._tracks.values() if not t.get('static')],
                cam_bearing(cx_for_assoc, self.p), self._lidar_id, self.p)
            if picked is not None:
                self._lidar_id = int(picked['id'])
                self._lidar_at = now
                self._blind_since = 0.0
                self._lidar_bearing = float(picked['bearing'])
                self._lidar_r = float(picked['r'])
                self._person_mem = {'x': picked.get('ox', 0.0),
                                    'y': picked.get('oy', 0.0),
                                    'vx': picked.get('vx', 0.0),
                                    'vy': picked.get('vy', 0.0), 'at': now}
        elif (sel >= 0 and self._lidar_id >= 0
              and self._person_mem is not None
              and (self._blind_since == 0.0
                   or (now - self._blind_since) <= self.p['lidar_hold_max_s'])
              and (now - self._person_mem['at'])
                  <= max(self.p['lidar_hold_s'], self.p['lidar_search_s'])):
            tr = self._tracks.get(self._lidar_id)
            # 同一条被点名的轨迹允许 static：人站着等车时 leg_tracker 会标 static，
            # 旧写法直接不跟 → 表现为"雷达跟踪突然断开"（2026-11-24 用户报）。
            if (tr is not None and (now - tr['at']) <= self.p['lidar_fresh_s']
                    and (not tr.get('static')
                         or int(tr['id']) == self._lidar_id)):
                lidar_target = tr
            else:
                # 轨迹换号/断了：按"刚才那个位置"重新认领（相机还看不到，只能靠位置）。
                # 2026-11-24 用户报"雷达跟着跟着突然断开"——实流里 leg_tracker 的 id
                # 在腿交叉/两腿合并时会重生，旧写法只认固定 id，一换号就丢。
                fresh = [t for t in self._tracks.values()
                         if (now - t['at']) <= self.p['lidar_fresh_s']]
                re_tr = reacquire_track(fresh, self._lidar_bearing,
                                        self._lidar_r, self.p, self._lidar_id)
                if re_tr is not None:
                    if int(re_tr['id']) != self._lidar_id:
                        self.get_logger().info(
                            '雷达轨迹换号 %d → %d（相机看不到，按位置续认）'
                            % (self._lidar_id, int(re_tr['id'])),
                            throttle_duration_sec=2.0)
                    self._lidar_id = int(re_tr['id'])
                    # ⚠️ 故意**不**刷新 self._lidar_at：续认只解决"换号"，
                    # 不延长"相机完全看不到"的最长窗口（那是一条安全边界）。
                    lidar_target = re_tr
                else:
                    # 相机背书的新轨迹也没有 → 按"人应该在的位置"认领**一条正在走动的轨迹**。
                    # 这一条专门解决 2026-11-26 用户报的"跟丢之后我还在旁边走也不认"：
                    # `pick_lidar_track` 只认相机背书，人绕到车侧/车后就永远认不回来。
                    # 安全边界：必须是**在走动**的轨迹 + 落在预测位置附近（家具不动）。
                    pred = predict_person(self._person_mem, now, self.p)
                    ad = adopt_moving_track(
                        [t for t in self._tracks.values()
                         if (now - t['at']) <= self.p['lidar_fresh_s']],
                        pred, now - self._person_mem['at'], self.p)
                    if ad is not None:
                        self._lidar_id = int(ad['id'])
                        lidar_target = ad
                        self.get_logger().info(
                            '按预测位置认领走动轨迹 #%d（距预测 %.2f m，相机看不到人）'
                            % (int(ad['id']),
                               math.hypot(float(ad['ox']) - pred[0],
                                          float(ad['oy']) - pred[1])),
                            throttle_duration_sec=3.0)

        # 续跟/认领来的目标如果**停住不动又没相机背书**，很可能根本不是人（家具/货箱）：
        # 短暂静止（人站着等车）先容忍，超过 `lidar_static_drop_s` 就放弃，回落成"停车/原地找"。
        if lidar_target is not None and lidar_target.get('static') \
                and not lidar_target.get('cam'):
            if self._lidar_static_since == 0.0:
                self._lidar_static_since = now
            elif (now - self._lidar_static_since) > self.p['lidar_static_drop_s']:
                self.get_logger().info(
                    '雷达目标静止 %.1f s 且无相机背书 → 放弃（疑似家具）'
                    % (now - self._lidar_static_since), throttle_duration_sec=3.0)
                lidar_target = None
                self._lidar_id = -1
                self._lidar_static_since = 0.0
        else:
            self._lidar_static_since = 0.0

        v, w, reason, box_h = decide(
            person, lost_s, self.clear_hard, self.clear_slow,
            self.left, self.right, self.p, size_hold=self._size_hold,
            rear_hard=self.rear_hard, rear_slow=self.rear_slow,
            back_hold=self._back_hold,
            gate_hold=(self._gate_hold
                       or (self._exit_gate and not self.door_is_clear)),
            gate_center=(self.gate_center if self._gate_hold
                         else self.gate_center_last),
            gate_exit=self._exit_gate, search=search_now,
            lost_side=self._last_side, err_dot=self.err_dot)

        # 重新认领后的软对准：误差已不大就先别满速修（掉头那种大误差照旧满速追）
        if (now < self._reacq_until and reason in ('facing', 'following')
                and self._ema['cx'] is not None):
            w = reacq_cap(w, float(self._ema['cx']) - 0.5, True,
                          self.reacq_wmax, self.reacq_soft_ex)

        # —— 雷达续跟接管（2026-11-24）——
        # 只在**相机已经丢人**（target_lost/no_target/target_gone）时接管；
        # 门口出门（gate_exit）、原地找人（search）这些"安全相关"的状态不许被覆盖。
        if lidar_target is not None and reason in ('target_lost', 'no_target',
                                                   'target_gone'):
            r_now = float(lidar_target['r'])
            if self._lidar_close:
                if r_now >= self.p['lidar_resume_m']:
                    self._lidar_close = False
            elif r_now <= self.p['lidar_stop_m']:
                self._lidar_close = True
            self._lidar_seen = now
            b_raw = float(lidar_target['bearing'])
            if self._lidar_bearing_trusted(b_raw, int(lidar_target['id']), now):
                b_used = self._lidar_bearing_filter(b_raw, now)
            else:
                # 突变还没确认：继续按上一次可信方位走（不跟着跳，也不急停）
                b_used = (self._lidar_bearing if self._lidar_b_s is None
                          else self._lidar_b_s)
                self.get_logger().info(
                    '雷达方位突变 %+.0f°（轨迹 #%d）→ 等确认，先按 %+.0f° 走'
                    % (math.degrees(b_raw), int(lidar_target['id']),
                       math.degrees(b_used)), throttle_duration_sec=2.0)
            self._lidar_bearing = b_used
            self._lidar_r = r_now
            # 滞回：先看是否该进 face，再看是否该退出（退出阈值更低）
            if abs(b_used) > self.p['lidar_face_rad']:
                self._lidar_face_hold = True
            elif abs(b_used) < self.p['lidar_face_exit_rad']:
                self._lidar_face_hold = False
            if 'ox' in lidar_target:
                # ⚠️ 别写进 `_person_mem`：那是**相机真值**，也是"按预测位置认领"的依据。
                # 拿雷达（可能是错目标）的数据覆盖它，会一路自我强化到错的人身上
                # （2026-11-26 用户报"雷达跟错人"）。雷达自己的记忆单独存。
                self._lidar_mem = {'x': float(lidar_target['ox']),
                                   'y': float(lidar_target['oy']),
                                   'vx': float(lidar_target.get('vx') or 0.0),
                                   'vy': float(lidar_target.get('vy') or 0.0),
                                   'at': now}
            v, w, reason = lidar_follow(
                b_used, r_now,
                self.clear_hard, self.clear_slow, self.left, self.right,
                self.p, close_hold=self._lidar_close,
                bearing_rate=self._lidar_rate,
                face_hold=self._lidar_face_hold)
            self.src = 'lidar'
            box_h = box_h or 0.0
            blind_age = ((now - self._blind_since) if self._blind_since else 0.0)
            backed = lidar_backed(lidar_target, self.p)
            if v > 0.0 and not lidar_drive_allowed(blind_age, self.p):
                # 雷达分不清"这个人"和"前面那个人"：盲跟超时后只许原地转向找人，
                # 不许再往前开（2026-11-26 用户报"人已经走过去了车还往前追"）。
                v = 0.0
                reason = 'lidar_blind_hold'
                self.get_logger().info(
                    '相机已丢 %.1f s，雷达只转不走（轨迹 #%d，%.2f m，%s）'
                    % (blind_age, self._lidar_id, r_now,
                       '相机背书' if backed else '无背书'),
                    throttle_duration_sec=3.0)
            self.get_logger().info(
                '雷达接管中 #%d 方位 %+.0f° 距离 %.2f m 轨迹速度 %.2f m/s '
                '相机背书=%s 盲跟 %.1f s'
                % (self._lidar_id, math.degrees(float(lidar_target['bearing'])),
                   r_now, float(lidar_target.get('v') or 0.0),
                   '是' if backed else '否', blind_age),
                throttle_duration_sec=3.0)
        elif (person is None and sel >= 0 and self._lidar_id >= 0
              and self._lidar_bearing is not None
              and (now - max(self._lidar_seen, self._lidar_at))
                  <= self.p['lidar_search_s']
              and reason in ('target_lost', 'no_target', 'target_gone')):
            # 雷达轨迹刚断（换号/两腿合并/被挡）：**原地按最后方位找人**，别直接停车。
            # 只转不走（v=0），扫到轨迹后下一帧就走正常的雷达接管。
            v, w, reason = lidar_search_cmd(self._lidar_bearing, self.p)
            if abs(w) > 1e-6:
                box_h = box_h or 0.0
                self.get_logger().info(
                    '雷达轨迹短暂丢失，原地按最后方位找人（%.0f°）'
                    % math.degrees(self._lidar_bearing),
                    throttle_duration_sec=3.0)
        elif self.src == 'cam':
            self._lidar_close = False        # 相机看回来，雷达侧迟滞复位
            self._lidar_face_hold = False
            if self._lidar_b_s is not None and (now - self._lidar_seen) > 2.0:
                self._lidar_b_s = None
                self._lidar_rate = 0.0
                self._lidar_last_ok = None
                self._lidar_flip = None

        # 平滑：加速/减速都限幅（避免一帧跳变把车"顿"一下）；
        # 安全类原因（障碍/丢人/太近）立刻归零，不做斜坡。
        # 'rear_blocked'/'facing' 也是"原地不动/只转"，线速度立即归零
        # 'search' 也是 v=0 但**要转**（每 tick 按方向给 w，别被归零）
        hard_stop = reason in ('obstacle_stop', 'target_lost', 'target_gone',
                              'too_close', 'no_target', 'rear_blocked', 'facing',
                              'gate_align', 'search', 'lidar_search',
                              # 雷达接管时 v 必须为 0 的几种（还要转身，所以不进下面的
                              # 显式归零列表）
                              'lidar_face', 'lidar_close', 'lidar_far',
                              'lidar_blind_hold', 'side_blocked')
        if hard_stop:
            self.v_cmd = 0.0
        else:
            dv = self.v_accel * dt
            self.v_cmd += max(-dv, min(dv, v - self.v_cmd))
        # 左右摆自动阻尼：只在"跟着人走"的阶段生效（原地对准/找人要满速转，别收）。
        # 收的是**目标**角速度，再由下面的斜坡限幅平滑，所以不会跳变。
        if reason in ('following', 'obstacle_slow', 'lidar_follow', 'lidar_slow',
                      'lidar_face', 'lidar_blind_hold',
                      # 2026-11-26：原地对准（facing）也会摆，一并交给自动阻尼。
                      # 它只"收力"，不会让车不转（最多收到 35%）。
                      'facing'):
            self.osc_gain = self._osc.update(w, dt)
        else:
            self.osc_gain = self._osc.update(0.0, dt)
        w = w * self.osc_gain
        dw = self.w_accel * dt
        self.w_cmd += max(-dw, min(dw, w - self.w_cmd))
        if hard_stop and reason in ('obstacle_stop', 'target_lost', 'target_gone',
                                    'no_target'):
            self.w_cmd = 0.0          # 这些必须完全停；facing/rear_blocked 还要转

        if reason == 'target_gone':
            self.get_logger().warn('目标丢失超过 %.0f s，跟随停止（请重新选人）'
                                   % self.p['lost_abort_s'], throttle_duration_sec=5.0)
        # ⚠️ 必须过一遍底盘硬限：|ω| ≤ 0.75、单轮 |v| + |ω|·track/2 ≤ manual_max_speed_mps(0.30)。
        # 超了 base_bridge 会整帧丢弃（Rejected out-of-range manual velocity），
        # 表现就是"页面显示正在转向，车却不动"（2026-11-21 实机踩坑）。
        pv, pw = limit_pair(self.v_cmd, self.w_cmd, self.p)
        self._publish(pv, pw)

        # "发了指令但车没动"：命令 |v|>0.06 持续 1.5 s、里程计却几乎为 0
        # → 九成是底盘把整帧丢了（超限/运动门/急停/传感器没就绪）。只报警不干预。
        odom_fresh = (now - self._odom_at) < 1.0
        if odom_fresh and abs(pv) > 0.06 and abs(self.odom_v) < 0.03:
            self._nomotion_s += dt
        else:
            self._nomotion_s = 0.0
        was = self.nomotion
        self.nomotion = self._nomotion_s > 1.5
        if self.nomotion and not was:
            self.get_logger().warn(
                '指令已发但车没动：v 命令 %.3f m/s，里程计 %.3f m/s（查运动门/急停/底盘硬限）'
                % (pv, self.odom_v), throttle_duration_sec=10.0)

        self._seen_prev = person is not None
        self._state(reason, pv, pw, box_h)

    def destroy_node(self):
        self._publish(0.0, 0.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:                        # noqa: BLE001
            pass
        # ⚠️ 只能关一次：SIGTERM 时 rclpy 信号处理器已 rcl_shutdown，再关会抛 RCLError
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
