#!/usr/bin/env python3
"""回归冒烟测试：每次改完代码/参数后跑一遍，确认「之前能用的功能还好用」。

用法：
  python3 tools/smoke_test.py nav     # 导航模式（已启动 use_saved_map:=true）
  python3 tools/smoke_test.py map     # 建图模式（已启动 real_auto_mapping）

检查项：
  1. 关键话题真的有数据（直接订阅，不用 ros2 topic hz —— 它在这个 DDS 配置下不可靠）
  2. 关键节点各只有 1 个（防重复启动）
  3. 全局代价地图尺寸 ≈ 地图尺寸（防"只剩 5×5 m 滚动窗口"这类回归）
  4. 能规划出路径，且路径离障碍 >= 0.25 m
  5. 运动链末端 /cmd_vel_auto_safe 有数据（防碰撞监控卡死）
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import label as cc_label
import tf2_ros
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

FAILS = []


def ok(msg):
    print(f'  ✓ {msg}')


def bad(msg):
    print(f'  ✗ {msg}')
    FAILS.append(msg)


def wait_for(node, topic, typ, timeout=8.0):
    got = []
    sub = node.create_subscription(typ, topic, lambda m: got.append(m), 10)
    start = time.time()
    while time.time() - start < timeout and not got:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got[-1] if got else None


def wait_transient(node, topic, timeout=10.0):
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    got = []
    sub = node.create_subscription(OccupancyGrid, topic, lambda m: got.append(m), qos)
    start = time.time()
    while time.time() - start < timeout and not got:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got[-1] if got else None


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'nav'
    rclpy.init()
    n = Node('smoke_test')
    print(f'== 冒烟测试（模式 {mode}）==')

    # 1) 话题
    for topic, typ in (('/scan', LaserScan), ('/tf', TFMessage)):
        if wait_for(n, topic, typ):
            ok(f'{topic} 有数据')
        else:
            bad(f'{topic} 没数据')

    # 2) 节点唯一性
    names = []
    for _ in range(3):                    # 节点发现有时序抖动，重试几次
        names = [name for name, _ in n.get_node_names_and_namespaces()]
        if all(any(t in x for x in names)
               for t in ('base_bridge', 'ekf_filter', 'sllidar', 'dm_imu')):
            break
        end = time.time() + 1.5
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
    for target in ('base_bridge', 'ekf_filter_node', 'sllidar', 'dm_imu'):
        c = sum(1 for x in names if target in x)
        if c == 1:
            ok(f'{target} 唯一')
        else:
            bad(f'{target} 有 {c} 个实例')

    # 3) 地图 vs 全局代价地图
    m = wait_transient(n, '/map')
    c = wait_transient(n, '/global_costmap/costmap')
    if m is None:
        bad('/map 拿不到')
    elif c is None:
        bad('/global_costmap/costmap 拿不到')
    else:
        mw = m.info.width * m.info.resolution
        mh = m.info.height * m.info.resolution
        cw = c.info.width * c.info.resolution
        ch = c.info.height * c.info.resolution
        print(f'    地图 {mw:.1f}x{mh:.1f} m   全局代价图 {cw:.1f}x{ch:.1f} m')
        if cw < mw * 0.9 or ch < mh * 0.9:
            bad('全局代价地图明显小于地图（静态层没吃到地图？）')
        else:
            ok('全局代价地图尺寸正常')
        check_plan(n, m)

    # 5) 运动链
    from geometry_msgs.msg import Twist
    if wait_for(n, '/cmd_vel_auto_safe', Twist, timeout=6.0):
        ok('/cmd_vel_auto_safe 有数据')
    else:
        bad('/cmd_vel_auto_safe 没数据（碰撞监控卡死？跑 tools/chain_watchdog.py）')

    rclpy.shutdown()
    print()
    if FAILS:
        print(f'== 有 {len(FAILS)} 项失败 ✗ ==')
        return 1
    print('== 全部通过 ✓ ==')
    return 0


def check_plan(n, m):
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    d = np.array(m.data).reshape(m.info.height, m.info.width)
    free = (d == 0)

    # 机器人在地图里的位置（TF）
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    rx = ry = None
    start = time.time()
    while time.time() - start < 8.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            tr = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            rx, ry = tr.transform.translation.x, tr.transform.translation.y
            break
        except Exception:                       # noqa: BLE001
            continue
    if rx is None:
        bad('拿不到 map→base_footprint（AMCL 没定位？）')
        return
    ok(f'机器人在地图 ({rx:.2f}, {ry:.2f})')

    # 选「机器人所在连通空闲区」里、离机器人 2.5~4 m 的一个点
    lab, _ = cc_label(free, structure=np.ones((3, 3)))
    cxp, cyp = int((rx - ox) / res), int((ry - oy) / res)
    if not (0 <= cxp < m.info.width and 0 <= cyp < m.info.height):
        bad('机器人位置超出地图'); return
    region = lab[cyp, cxp]
    if region == 0:
        # 机器人脚下不是空闲格（可能贴墙），找 1 m 内最近的空闲格
        ys, xs = np.where(lab > 0)
        if len(ys) == 0:
            bad('地图没有空闲格'); return
        dist = (ys - cyp) ** 2 + (xs - cxp) ** 2
        idx = int(np.argmin(dist))
        cyp, cxp = int(ys[idx]), int(xs[idx])
        region = lab[cyp, cxp]
        ok('机器人脚下非空闲，改用最近空闲格做起点')
    cand = np.argwhere(lab == region)
    dist_m = np.hypot((cand[:, 0] - cyp) * res, (cand[:, 1] - cxp) * res)
    pick = None
    for lo, hi in ((2.5, 4.0), (1.5, 6.0), (0.5, 10.0)):
        sel = cand[(dist_m >= lo) & (dist_m <= hi)]
        if len(sel):
            pick = sel[len(sel) // 2]
            break
    if pick is None:
        bad('找不到合适的测试目标'); return
    gy, gx = int(pick[0]), int(pick[1])
    gx_m, gy_m = ox + gx * res, oy + gy * res
    ok(f'测试目标 ({gx_m:.2f}, {gy_m:.2f})，距机器人 '
       f'{math.hypot(gx_m - rx, gy_m - ry):.1f} m（同连通区）')

    # 机器人脚下可能是致命格（贴近墙/膨胀），规划器无法从致命格起步。
    # 这时显式给一个可通行的起点，测试的是"规划器+代价地图"本身。
    cm = wait_transient(n, '/global_costmap/costmap')
    start_xy = (float(rx), float(ry))
    if cm is not None:
        cres = cm.info.resolution
        cox, coy = cm.info.origin.position.x, cm.info.origin.position.y
        cdata = np.array(cm.data).reshape(cm.info.height, cm.info.width)

        def cost(x, y):
            cx_, cy_ = int((x - cox) / cres), int((y - coy) / cres)
            if 0 <= cx_ < cm.info.width and 0 <= cy_ < cm.info.height:
                return int(cdata[cy_, cx_])
            return 255

        if cost(*start_xy) >= 99:
            best = None
            for r in range(1, 40):
                for k in range(24):
                    a = 2 * math.pi * k / 24
                    qx = rx + r * cres * math.cos(a)
                    qy = ry + r * cres * math.sin(a)
                    if cost(qx, qy) < 99:
                        best = (qx, qy)
                        break
                if best:
                    break
            if best:
                start_xy = best
                ok(f'机器人脚下代价 {cost(rx, ry)} 不可通行，改用附近可通行起点 '
                   f'({best[0]:.2f}, {best[1]:.2f})')
            else:
                bad('机器人附近找不到可通行格')

    cli = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    if not cli.wait_for_server(timeout_sec=10.0):
        bad('compute_path_to_pose 服务不可用')
        return
    goal = ComputePathToPose.Goal()
    goal.goal = PoseStamped()
    goal.goal.header.frame_id = 'map'
    goal.goal.pose.position.x = float(gx_m)
    goal.goal.pose.position.y = float(gy_m)
    goal.goal.pose.orientation.w = 1.0
    goal.use_start = True
    goal.start = PoseStamped()
    goal.start.header.frame_id = 'map'
    goal.start.pose.position.x = start_xy[0]
    goal.start.pose.position.y = start_xy[1]
    goal.start.pose.orientation.w = 1.0
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        bad('规划目标被拒绝')
        return
    rf = handle.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=25.0)
    res_msg = rf.result()
    if res_msg is None or res_msg.status != 4:
        bad('规划失败（找不到路径）')
        return
    path = res_msg.result.path
    occ = np.argwhere(d > 50)
    mind = 9.9
    for ps in path.poses:
        px, py = ps.pose.position.x, ps.pose.position.y
        px_i, py_i = int((px - ox) / res), int((py - oy) / res)
        if len(occ) == 0:
            break
        sub = occ[(np.abs(occ[:, 0] - py_i) < 40) & (np.abs(occ[:, 1] - px_i) < 40)]
        for (yy, xx) in sub:
            dd = math.hypot((yy - py_i) * res, (xx - px_i) * res)
            if dd < mind:
                mind = dd
    ok(f'规划成功：{len(path.poses)} 点，离障碍最近 {mind:.2f} m')
    if mind < 0.25:
        bad('路径离障碍太近（<0.25 m）')


if __name__ == '__main__':
    sys.exit(main())
