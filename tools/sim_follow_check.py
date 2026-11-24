#!/usr/bin/env python3
"""跟随节点的黑盒测试（隔离 ROS_DOMAIN_ID 里跑，不碰实机、不发运动指令到真车）。

跑法（在机器人或任何有 ROS 的机器上）：

    export ROS_DOMAIN_ID=42
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    ./.venv/bin/python -m fwr_bringup.person_follower &     # 被测节点
    ./.venv/bin/python tools/sim_follow_check.py            # 本脚本

场景（2026-11-21 第二版：框大小判远近 + 走廊避障 + 转向更快 + 0.25 m/s）
  A 框 55%（中等）、前方空旷      → v 收敛到 0.29、w≈0
  B 人偏右                        → w ≤ -0.5（比第一版 0.44 更快）
  C 走廊内 0.30 m 有障碍          → v=0（obstacle_stop）
  D 心跳停发                      → 2 s 内 v=0
  E 门框（±35°/0.5 m）挡住视线    → **不算障碍**，v 仍 >0（能出门）
  F 框 80%（到停距）              → v=0（too_close）
  G 检测器掉帧 0.4 s（只发雷达）  → **不像急刹**，沿用上一帧框继续走（v>0.15）
  H 框 95% + 车尾空（人贴脸）     → **v<0 后退**（backing）
  I 框 95% + 车尾 0.15 m          → v=0 但继续转（rear_blocked，原地对准）
  J 人贴画面右边（从车旁走过）    → v=0、w<0（facing，车头追着框转）
  K 正对门洞（净宽 0.9 m）        → gate_pass，v≈0.12、|w| 小（不再跟着框摆）
  L 门洞偏左 30 cm                → gate_align，v=0、w>0（先原地站正再进）
  M 门洞里人偏到画面右边          → 仍然 gate_pass（**不跟框转**，这是卡门框的根因）
  N 门里丢人（人先出门了）        → **gate_exit 继续往外走**，不停在门框中间
  O 出门后还看不到人              → **search 原地转身找人**（人在右边 → w<0 右转）
  Q2 门线只到车后 0.34 m          → **继续直出**（旧判据会在这里就转身蹭门框）
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rclpy.init()
node = Node('follow_check')
pub_p = node.create_publisher(String, '/robot/vision/persons', 10)
pub_s = node.create_publisher(LaserScan, '/scan_merged', qos_profile_sensor_data)
pub_e = node.create_publisher(String, '/follow/enable', 10)
pub_l = node.create_publisher(String, '/robot/lidar/persons', 10)   # P2 雷达轨迹

pub_o = node.create_publisher(Odometry, '/odom', 10)
state = {'v': 0.0, 'w': 0.0, 't': 0.0, 'reason': '', 'nomotion': False}
ODOM_V = {'v': 0.0}


def send_odom():
    m = Odometry()
    m.twist.twist.linear.x = ODOM_V['v']
    pub_o.publish(m)


def on_cmd(msg):
    state['v'], state['w'], state['t'] = msg.linear.x, msg.angular.z, time.monotonic()


def on_follow_state(msg):
    try:
        data = json.loads(msg.data or '{}')
    except Exception:
        return
    state['reason'] = data.get('reason', state.get('reason', ''))
    state['nomotion'] = bool(data.get('nomotion'))
    state['osc_gain'] = data.get('osc_gain')
    state['src'] = data.get('src')
    state['lidar_id'] = data.get('lidar_id')


# 必须等 on_follow_state 定义之后再订阅（模块是自上而下执行的）
node.create_subscription(String, '/follow/state', on_follow_state, 10)


def on_state(msg):
    try:
        state['reason'] = json.loads(msg.data).get('reason', '')
    except Exception:                                  # noqa: BLE001
        pass


node.create_subscription(Twist, '/cmd_vel_manual', on_cmd, 10)
node.create_subscription(String, '/follow/state', on_state, 10)


def make_scan(points):
    """points = [(角度, 距离)]，其余 30 m 空旷。"""
    sc = LaserScan()
    sc.angle_min, sc.angle_max = -math.pi, math.pi
    sc.angle_increment = 2 * math.pi / 360
    sc.range_min, sc.range_max = 0.05, 12.0
    rng = [30.0] * 360
    for deg, r in points:
        idx = int(round((deg + 180) / 1.0)) % 360
        rng[idx] = r
    sc.ranges = rng
    return sc


def cart(points):
    """[(纵向 lon, 横向 lat)] → [(角度 deg, 距离 m)]，写门框位置更直观。"""
    return [(math.degrees(math.atan2(lat, lon)), math.hypot(lon, lat))
            for lon, lat in points]


def send(box_x=0.42, box_h=0.55, points=None, sel=1, tracking=True, enable=True):
    box = {'id': sel, 'x': box_x, 'y': 0.25, 'w': 0.14, 'h': box_h, 's': 0.9}
    pub_p.publish(String(data=json.dumps({'w': 640, 'h': 480, 'persons': [box],
                                          'sel': sel, 'tracking': tracking,
                                          'lost_s': 0.0})))
    pub_s.publish(make_scan(points or []))
    if enable:
        pub_e.publish(String(data=json.dumps({'on': True})))
    ODOM_V['v'] = 0.25 if ODOM_V.get('ok', True) else 0.0
    send_odom()


def send_lost(points=None, lost_s=1.5, sel=1):
    """发布"目标丢失"帧（persons 为空 + tracking False），门框点照发。"""
    pub_p.publish(String(data=json.dumps({'w': 640, 'h': 480, 'persons': [],
                                          'sel': sel, 'tracking': False,
                                          'lost_s': lost_s})))
    pub_s.publish(make_scan(points or []))
    pub_e.publish(String(data=json.dumps({'on': True})))


def send_lidar(tracks):
    """tracks = [(id, x, y)] 或 [(id, x, y, static)]，odom 系（位姿恒为原点）。"""
    payload = {'tracks': [{'id': int(t[0]), 'x': float(t[1]), 'y': float(t[2]),
                           'vx': 0.0, 'vy': 0.0, 'age_s': 1.0, 'misses': 0,
                           'cam': False,
                           'static': bool(t[3]) if len(t) > 3 else False}
                          for t in (tracks or [])]}
    pub_l.publish(String(data=json.dumps(payload)))


def spin_p2(sec, box_x=None, box_h=0.55, points=None, sel=1,
            tracks=None, enable=True):
    """P2 混合帧：相机框（box_x=None 表示"相机看不到人"）+ 雷达轨迹 + 扫描。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        if box_x is None:
            pub_p.publish(String(data=json.dumps(
                {'w': 640, 'h': 480, 'persons': [], 'sel': sel,
                 'tracking': False, 'lost_s': 1.5})))
        else:
            box = {'id': sel, 'x': box_x, 'y': 0.25, 'w': 0.14, 'h': box_h,
                   's': 0.9}
            pub_p.publish(String(data=json.dumps(
                {'w': 640, 'h': 480, 'persons': [box], 'sel': sel,
                 'tracking': True, 'lost_s': 0.0})))
        send_lidar(tracks)
        pub_s.publish(make_scan(points or []))
        if enable:
            pub_e.publish(String(data=json.dumps({'on': True})))
        ODOM_V['v'] = 0.25
        send_odom()
        rclpy.spin_once(node, timeout_sec=0.02)
    rclpy.spin_once(node, timeout_sec=0.05)
    return state['v'], state['w']


def spin_lost(sec, **kw):
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        send_lost(**kw)
        rclpy.spin_once(node, timeout_sec=0.02)
    rclpy.spin_once(node, timeout_sec=0.05)
    return state['v'], state['w']


def spin(sec, **kw):
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        send(**kw)
        rclpy.spin_once(node, timeout_sec=0.02)
    rclpy.spin_once(node, timeout_sec=0.05)
    return state['v'], state['w']


results = []
v, w = spin(2.5, box_x=0.42, box_h=0.55)
results.append(('A 框55% 空旷', v > 0.20 and abs(w) < 0.05, f'v={v:.3f} w={w:.3f}'))

v, w = spin(2.0, box_x=0.66, box_h=0.55)
# 第七版起跟随时 ω 上限 steer_wmax 0.42（PD + 限幅），不再要求 ±0.63
results.append(('B 人偏右', w <= -0.35, f'v={v:.3f} w={w:.3f}'))

v, w = spin(2.0, box_x=0.42, box_h=0.55, points=[(0, 0.30)])
results.append(('C 走廊0.30m障碍', abs(v) < 1e-6, f'v={v:.3f} w={w:.3f}'))

v, w = spin(2.0, box_x=0.42, box_h=0.90)
results.append(('F 框90% 到停距', abs(v) < 1e-6, f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# 框 80%：新阈值下应该**继续跟**，不再停（用户 2026-11-21 晚要求）
v, w = spin(2.0, box_x=0.42, box_h=0.80)
results.append(('F2 框80% 继续跟', v > 0.20, f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# 净宽 1.24 m 的门口（横向 ±0.62 m）：走廊外 → 不算障碍，也不算"窄通道"（>1.05 m）→ 正常速度
v, w = spin(2.0, box_x=0.42, box_h=0.55, points=[(37.8, 1.010), (-37.8, 1.010)])
results.append(('E 宽门口±0.62m 正常走', v > 0.20, f'v={v:.3f} w={w:.3f}'))

# G：检测器掉帧 0.4 s（YOLO 偶尔漏帧）。只发 enable + 雷达，不发 persons。
v0, _ = spin(1.5, box_x=0.42, box_h=0.55)
t0 = time.monotonic()
while time.monotonic() - t0 < 0.40:
    pub_e.publish(String(data=json.dumps({'on': True})))
    pub_s.publish(make_scan([]))
    rclpy.spin_once(node, timeout_sec=0.02)
rclpy.spin_once(node, timeout_sec=0.05)
v, w = state['v'], state['w']
results.append(('G 检测掉帧0.4s 不急刹', v > 0.15, f'v={v:.3f} (前 {v0:.3f})'))

# H：人贴脸（框 95%）+ 车尾空 → 应该**后退**，不是傻站着
v, w = spin(2.5, box_x=0.42, box_h=0.95)
results.append(('H 框95% 车尾空 → 后退', v < -0.05, f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# I：人贴脸 + 车尾 0.15 m 有东西 → 不许退，但车头继续转向对准
v, w = spin(2.5, box_x=0.58, box_h=0.95, points=[(180, 0.15)])
results.append(('I 车尾0.15m → 只原地转',
                abs(v) < 1e-6 and w < -0.1,
                f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# J：人从车旁走过（框贴到画面右边）→ 原地转向追框，不前进
v, w = spin(2.0, box_x=0.86, box_h=0.55)
results.append(('J 人贴边缘 → 原地追框',
                abs(v) < 1e-6 and w < -0.3,
                f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# K/L/M：穿门框（2026-11-21 第四版）。门框点：(角度°, 距离)，净宽 0.9 m 的门两边在 ±32.7°
GATE = [(32.7, 0.832), (-32.7, 0.832)]
v, w = spin(2.5, box_x=0.42, box_h=0.55, points=GATE)
results.append(('K 正对门洞 → 低速直穿',
                state['reason'] == 'gate_pass' and abs(w) < 0.2 and 0.0 < v <= 0.19,
                f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# 门洞偏左 10 cm：左框 0.55 / 右框 -0.35（净宽 0.90，车能过；中心 +0.10 > 0.07）
GATE_OFF = [(38.2, 0.890), (-26.6, 0.783)]
v, w = spin(2.5, box_x=0.42, box_h=0.55, points=GATE_OFF)
results.append(('L 门洞偏左 → 先站正',
                state['reason'] == 'gate_align' and abs(v) < 1e-6 and w > 0.1,
                f'v={v:.3f} w={w:.3f} {state["reason"]}'))

v, w = spin(2.5, box_x=0.80, box_h=0.55, points=GATE)
results.append(('M 门里人偏右 → 不跟框转',
                state['reason'] == 'gate_pass' and abs(w) < 0.2,
                f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# N：门口丢人（人比车快，先出门了）→ 不许停，要接着把门洞走完
spin(1.5, box_x=0.70, box_h=0.55, points=GATE)      # 人先在画面右边，记下方向
v, w = spin_lost(1.5, points=GATE, lost_s=1.5)
results.append(('N 门里丢人 → 继续出门',
                state['reason'] == 'gate_exit' and v > 0.05,
                f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# O：门洞过去了还看不到人 → 原地按最后方向找人（人在右边 → w<0 右转）
v, w = spin_lost(1.2, points=[], lost_s=2.0)
results.append(('O 出门后找人 → 右转',
                state['reason'] == 'search' and abs(v) < 1e-6 and w < -0.3,
                f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# 第六版：整车出门（前后雷达）。车头过了门线、车尾还在门里 → 不许转身找人
gate_ahead = cart([(0.70, 0.45), (0.70, -0.45)])
spin(1.2, box_x=0.42, box_h=0.55, points=gate_ahead)        # 先进门洞模式
v, w = spin_lost(2.0, points=cart([(-0.10, 0.45), (-0.12, -0.45)]))
results.append(('Q 车尾还在门里→继续直出', state['reason'] == 'gate_exit' and v > 0.10,
                f'v={v:.3f} w={w:.3f} {state["reason"]}'))

v, w = spin_lost(1.5, points=cart([(-0.45, 0.45), (-0.47, -0.45)]))
results.append(('R 整车出门→转身找人', state['reason'] == 'search' and abs(w) > 0.2,
                f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# Q2（2026-11-22 用户第二次报"没完全出来就转向"）：门线只退到车后 0.34 m。
# 旧判据 -0.29 会判"已出去"→ 转身；但原地转身时车尾角点扫到 -0.315、加余量要 -0.415，
# 所以这里**必须继续直出**（gate_exit），不许 search。
spin(1.2, box_x=0.42, box_h=0.55, points=gate_ahead)
v, w = spin_lost(2.0, points=cart([(-0.34, 0.45), (-0.36, -0.45)]))
results.append(('Q2 车后0.34m 未全出→继续直出',
                state['reason'] == 'gate_exit' and v > 0.10,
                f'v={v:.3f} w={w:.3f} {state["reason"]} thresh={state.get("door_thresh")}'))

# 第七版：目标左右摆 → 自动收转向增益；稳定后恢复（防"车一直左转右转"）
for i in range(80):
    # 每 8 拍（≈0.4 s）换一侧，相当于车以 ~1.2 Hz 左右摆
    spin(0.05, box_x=0.75 if (i // 8) % 2 == 0 else 0.25, box_h=0.55)
results.append(('S 目标左右摆→自动收力',
                float(state.get('osc_gain') or 1.0) < 0.70,
                f'osc_gain={state.get("osc_gain")} w={state["w"]:.3f}'))
v, w = spin(8.0, box_x=0.60, box_h=0.55)
results.append(('T 稳定跟随→自动恢复',
                float(state.get('osc_gain') or 0.0) > 0.90,
                f'osc_gain={state.get("osc_gain")} v={v:.3f}'))

# P：底盘整帧丢弃（里程计不动）→ /follow/state 必须报 nomotion（页面据此提示用户）
ODOM_V['ok'] = False
v, w = spin(2.6, box_x=0.42, box_h=0.55)
results.append(('P 车不动 → 报警', bool(state.get('nomotion')) and v > 0.05,
                f'v={v:.3f} nomotion={state.get("nomotion")}'))
# —— P2 相机 + 雷达融合（2026-11-24）——
# 用户要求「人绕到车侧/车后仍要跟」：相机视场 70° 看不到侧面，靠雷达续跟。
spin_p2(2.0, box_x=0.42, box_h=0.55, tracks=[(7, 2.0, 0.0)])     # 相机点名 + 雷达同一条
v, w = spin_p2(1.5, box_x=None, tracks=[(7, 2.0, 0.05)])         # 相机丢了，雷达还在
results.append(('P2-A 相机丢人→雷达续跟',
                str(state.get('src')) == 'lidar' and v > 0.05
                and str(state['reason']).startswith('lidar'),
                f'v={v:.3f} w={w:.3f} {state["reason"]} src={state.get("src")}'))

# 从没被相机点名过（sel=-1）的轨迹**绝不能**起跟
v, w = spin_p2(2.0, box_x=None, sel=-1, tracks=[(9, 2.0, 0.0)])
results.append(('P2-B 没被相机点名→不跟',
                abs(v) < 1e-6 and state.get('src') != 'lidar',
                f'v={v:.3f} {state["reason"]} src={state.get("src")}'))

# 相机丢 + 雷达轨迹也没了 → 停车（不留"幽灵目标"）
v, w = spin_p2(1.5, box_x=None, tracks=[])
results.append(('P2-C 雷达也丢→停车',
                abs(v) < 1e-6, f'v={v:.3f} {state["reason"]}'))

# 人绕到车后（雷达 171°）→ 原地转身对准，不前进
spin_p2(1.5, box_x=0.42, box_h=0.55, tracks=[(7, 2.0, 0.0)])
v, w = spin_p2(2.0, box_x=None, tracks=[(7, -2.0, 0.30)])
results.append(('P2-D 人在车后→原地转身',
                state['reason'] == 'lidar_face' and abs(v) < 1e-6 and w > 0.2,
                f'v={v:.3f} w={w:.3f} {state["reason"]}'))

# 雷达接管期间，车头走廊 0.30 m 有障碍 → 照样停（安全优先级最高）
spin_p2(1.5, box_x=0.42, box_h=0.55, tracks=[(7, 2.0, 0.0)])
v, w = spin_p2(1.5, box_x=None, tracks=[(7, 2.0, 0.0)], points=[(0, 0.30)])
results.append(('P2-E 接管时走廊0.3m→停',
                abs(v) < 1e-6, f'v={v:.3f} {state["reason"]}'))

# P2-F：雷达轨迹换号（实流里腿交叉会让 id 重生）→ 按位置续认，不许断
spin_p2(1.5, box_x=0.42, box_h=0.55, tracks=[(7, 2.0, 0.0)])
v, w = spin_p2(1.5, box_x=None, tracks=[(9, 2.02, 0.05)])   # id 7 → 9，位置几乎没变
results.append(('P2-F 雷达换号→按位置续认',
                str(state.get('src')) == 'lidar' and v > 0.05,
                f'v={v:.3f} {state["reason"]} src={state.get("src")}'))

# P2-G：同样的位置只有一条 static 轨迹（椅子）→ 不许被续认，应该停车
v, w = spin_p2(1.5, box_x=None, tracks=[(11, 2.0, 0.0, True)])
results.append(('P2-G 只剩家具(static)→不跟',
                abs(v) < 1e-6 and str(state.get('src')) != 'lidar',
                f'v={v:.3f} {state["reason"]} src={state.get("src")}'))

ODOM_V['ok'] = True

t0 = time.monotonic()
while time.monotonic() - t0 < 3.2:
    send(box_x=0.42, box_h=0.55, enable=False)
    rclpy.spin_once(node, timeout_sec=0.02)
rclpy.spin_once(node, timeout_sec=0.05)
v, w = state['v'], state['w']
results.append(('D 心跳停止2s', abs(v) < 1e-6 and abs(w) < 1e-6, f'v={v:.3f} w={w:.3f}'))

ok = True
for name, passed, detail in results:
    print(('PASS ' if passed else 'FAIL ') + name + '  ' + detail)
    ok = ok and passed
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
