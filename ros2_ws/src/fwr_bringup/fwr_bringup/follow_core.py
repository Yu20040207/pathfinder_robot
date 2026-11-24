"""跟随控制律（纯函数，便于单测；不含 ROS）。

2026-11-21 第二版：**按框的大小判断远近**，不再用身高估米数。

为什么换
--------
用 ``d = fy × 1.7 / h_px`` 估距离，人一走近框就被画面裁掉（h 永远停在 1.0），
估出来的距离卡在 ~2 m 不再变小 → 车**永远到不了"该停"的距离**，会一直往前顶。
用户实测就是这个现象。改成直接看**框高占画面的比例**：框越大 = 人越近，
阈值就是"框到多大就停"，跟相机内参、身高都无关。

输入
----
* ``person``：选中目标的框（归一化 ``x,y,w,h``）
* ``clear_hard`` / ``clear_slow``：雷达在**车身走廊**内的最近距离
  （走廊 = 横向 ±0.22 m，见 ``scan_metrics``）——车能过的门框不会被当成障碍
* ``left_m`` / ``right_m``：转身方向的侧向净空

输出
----
``(v, w, reason, box_h)``
"""

from __future__ import annotations

import math

DEFAULTS = {
    # —— 远近：只看框高占比（0~1）——
    # 2026-11-21 晚（用户第三次实测）：75% 就停显得"离得太远、停住等人"。
    # 改成 85% 才停、缩到 80% 立刻重新走 —— 迟滞只剩 5%，框到 80% 时车是在动的。
    'stop_box_h': 0.85,   # 框高 ≥ 85% → 停
    'resume_box_h': 0.81, # 停过之后框缩到 81% 就重新走（迟滞 4%，留 1% 余量：
                          # "框 80% 时必须在动"，卡在 0.80 正好是边界会犹豫）
    # ⚠️ 这个值越大，跟得越快：0.62 时"框 0.65~0.70"（最常见的跟随距离）只有
    # 0.19→0.10 m/s，用户 2026-11-21 反馈"太慢"。改成 0.70：框 ≤0.70 全速，
    # 0.70~0.75 才减速，减速窗口只剩 5%（靠 v_accel 0.6 平滑，不会顿）。
    'go_box_h': 0.82,
    'far_box_h': 0.30,    # 框高 ≤ 30%（≈6 m 外）→ 不追，只转过去看
    # —— 速度 / 转向（跟随专用，改完不影响导航）——
    # 上限由底盘硬限决定：manual_max_speed_mps 0.30 − |ω|·track/2 ⇒ 直行最多 0.29。
    # 2026-11-21 用户再说"太慢"：0.25 → 0.29（在不改底盘安全参数的前提下到顶）。
    'vmax': 0.29,
    # ⚠️ 必须 < 0.75：base_bridge.manual_velocity() 硬限 |ω| ≤ 0.75，超了整帧被丢
    # （日志 Rejected out-of-range manual velocity）→ 页面显示"正在转向"但车一动不动。
    # 第二版曾设 0.9，实机 facing/转弯时全被拒（2026-11-21 实测确认）。
    'wmax': 0.70,
    # 2026-11-21 晚（用户报"直线跟随左右摆"）：纯 P + 3.5 增益在慢底盘上必然极限环。
    # 改成 PD：kp 3.5 → 2.4，加 kd 阻尼，跟随阶段 ω 上限单独夹 0.42。
    # 参考开源 `Pramod-Gurav/ros2-rgbd-person-follower`（angular PID + EMA + 30 px 死区，
    # kp 0.004/px 折合满幅约 1.3 rad/s，比我们原来的 3.5 温和得多）。
    'yaw_kp': 2.4,        # 2.2 → 3.5，同样为跟得上人横向走动
    'yaw_kd': 0.9,        # 阻尼：d(框中心偏差)/dt 的增益（rad/s 每 1/s 偏差变化）
    'steer_wmax': 0.42,   # 跟随时 ω 上限（原地对准 facing 仍用 wmax 0.70）
    'yaw_deadband': 0.06,   # 画面中间 ±5% 不转（检测框每帧会抖，3% 太敏感）
    # —— 走廊避障（车身半宽 0.215，留 5 mm）——
    # 单轮速度包络：base_bridge manual_max_speed_mps=0.30，超了同样整帧被丢。
    # 留 1 cm/s 余量，命令按 |v| + |ω|·track/2 ≤ max_wheel 收（保曲率，收线速度）。
    'max_wheel_mps': 0.29,
    # ⚠️ 必须等于 `BaseBridgeCore().track_m`（2026-11-11 换 WSDC2412D-V3 + 100 mm 轮后
    # 实测 = **0.317**，旧值 0.262 作废）。用旧值算出的单轮包络偏小 1.3 cm/s，
    # 底盘会判超限**整帧丢弃** → 页面显示"跟随中"车不动（2026-11-21 实机踩坑，
    # 日志 11 次 `Rejected out-of-range manual velocity（v=0.228 w=0.476 …）`）。
    # 有单测 `test_track_m_matches_chassis` 钉死这条，别再手改。
    'track_m': 0.317,
    'corridor_m': 0.22,
    'front_hard_m': 0.35, # 走廊内 35 cm 有东西 → 停
    'front_slow_m': 0.60, # 走廊内 60 cm → 减速
    'side_clear_m': 0.30, # 要往哪边转，那边至少这么空
    # —— 目标丢失 ——
    # —— 侧向对准 / 后退（2026-11-21 第三版）——
    'face_ex': 0.30,        # 目标偏离画面中线 >30% → 原地转向对准（v=0）
    # —— 穿门框（2026-11-21 第四版，用户报"出门框卡在门框上"）——
    # 门框在车身走廊外（横向 0.4~0.6 m），走廊避障看不见它；跟框转向时车角扫过去就顶住。
    # 现在一旦在车前 1.1 m 内测到"窄通道"（两侧都有边界、净宽 ≤ 1.05 m），
    # 就**改瞄门洞中心、进门前先站正、门内限速直线走**，不再跟着人左右摆。
    'gate_look_m': 1.10,    # 多远的门洞算"要进门了"
    'gate_half_span_m': 1.20,  # 只在横向 ±1.2 m 内找门框
    'gate_min_gap_m': 0.55,    # 比这还窄（车宽 0.43）不算门洞，交给走廊避障
    'gate_wide_m': 1.05,    # 比这宽就不算"窄通道"（走廊 1.2 m 不触发，仍正常跟人）
    'gate_align_m': 0.07,   # 门洞中心偏超过 7 cm → 先原地站正（v=0）
    'gate_kp': 2.5,         # 对中增益（比跟框软，避免门内左右摆）
    'gate_wmax': 0.45,      # 门内转向限幅（跟框是 0.70）
    'gate_vmax': 0.18,      # 门内限速（0.12 → 0.18，用户反馈整体太慢）
    'gate_hold_s': 0.35,    # 门洞看不到后再保持 0.35 s（防抖）
    'align_first_m': 0.80,  # 车头 0.80 m 内有东西时，先对准再走（防切内角撞门框）
    'align_first_ex': 0.12,
    # —— 门内丢人 → 先出门再找人（2026-11-21 用户要求）——
    # 人比车快，出门时人先消失在画面里；以前一丢人就停车，卡在门框中间。
    # 现在：门洞还在 + 丢了人 → 先把门洞走完（gate_exit），出门口后原地按
    # "最后看到的方向"转身找人（search）。人一回来立刻恢复正常跟随。
    # 2026-11-21 晚（用户："前雷达出去就算出门了？整车出去才能转"）：
    # 车长 ≈0.46 m（后雷达在 -0.229），前雷达刚过门线时**车尾还在门里**。
    # 用 360° 合并扫描量"门线相对车身"的纵向位置，门线退到车尾后面才算整车出去。
    'body_rear_m': 0.23,       # 车尾到 base_link 的距离（URDF 后雷达 -0.229）
    'body_half_width_m': 0.215,  # 车身半宽（URDF 车体 0.43 m 宽）
    'door_clear_margin_m': 0.10,  # 门线退到"车身旋转扫过的圆"后方 10 cm 才算出去
    'door_lost_hold_s': 0.60,   # 门线一时没测到（遮挡/出视场）时，最多沿用上一次测量这么久
    'door_lat_min_m': 0.28,    # 找门框点的横向窗口（走廊外、门宽内）
    'door_lat_max_m': 0.80,
    'gate_exit_enabled': True,
    'gate_exit_max_s': 4.0,    # 最多顶着丢人往外走这么久（防止卡在门口一直挪）
    'search_turn_s': 2.5,      # 出门后原地找人转多久（|ω|0.5 → 约 72°）
    'search_wmax': 0.50,
    'back_box_h': 0.92,     # 框高 ≥ 92% → 人贴脸了，往后退
    'back_release_box_h': 0.85,   # 退到 85% 才停（迟滞）
    'back_vmax': 0.12,      # 后退速度（比前进慢）
    'back_hard_m': 0.30,    # 车尾走廊 0.30 m 内有东西 → 不许退，只原地转
    'back_slow_m': 0.60,    # 0.60 m 内开始减速后退
    'lost_stop_s': 0.8,
    'lost_abort_s': 3.0,
    # —— P2 相机 + 雷达融合（2026-11-24）——
    # 相机看不到人时（人绕到车侧/车后），靠雷达把"这一个人"续跟一段时间。
    # ⚠️ 安全铁律：**雷达不许自己起跟**。只有刚才被相机点名的人（同一个雷达 id）
    # 还在，才允许接管；从没被相机看到过的轨迹一律不跟（家具/路人）。
    'lidar_hold_s': 8.0,        # 相机丢掉后，雷达最多续跟这么久（原 5 s，实测偏短）
    # 2026-11-24：`leg_tracker` 的 id 在实流里不稳（腿交叉/两腿合并/家具干扰都会重生），
    # 相机看不到时 id 一换就等于"目标丢了" → 现场就是"雷达跟着跟着突然断开"。
    # 换号后按"刚才那个位置"重新认领（只用下面这两个门限，别放宽）。
    'lidar_reacq_deg': 20.0,    # 重新认领：方位差上限（°）
    'lidar_reacq_dr': 0.90,     # 重新认领：距离差上限（m）
    'lidar_fresh_s': 0.5,       # 雷达轨迹多久没更新就算过期（10 Hz 扫描，0.5 s 够宽）
    'lidar_assoc_deg': 25.0,    # 相机看到人时，用它对准雷达轨迹（方位差上限）
    'cam_hfov_deg': 70.0,       # 相机水平视场（HP60C 实测约 70°），框中心 → 方位角
    'lidar_stop_m': 1.20,       # 雷达目标近到这个距离 → 停（相当于"框 85%"）
    'lidar_resume_m': 1.55,     # 停过之后退到这么远才重新走（迟滞 35 cm）
    'lidar_far_m': 3.00,        # 超过这个距离只转不追（看不清是腿还是家具）
    'lidar_face_rad': 0.50,     # 目标偏离车头超过 29° → 原地转向对准，不前进
    'lidar_bearing_deadband': 0.05,   # ±3° 不转（雷达有抖动）
    'lidar_kp': 1.8,            # 方位误差 → 角速度
    'lidar_wmax': 0.42,         # 雷达接管时的转向上限（与跟框 steer_wmax 一致）
    'lidar_vmax': 0.22,         # 雷达接管时的前进上限（没相机辅助，保守一点）
}


def scan_metrics(ranges, angle_min: float, angle_increment: float,
                 p: dict | None = None):
    """把雷达扫成"车身走廊"指标：车头净空 + 车尾净空 + 左右侧净空。

    为什么不用角度锥：门框在门两侧（0.5 m 处 ±35°、横向只有 0.29 m），
    用 ±30° 锥会把门框当障碍 → 人出门后车出不了门（2026-11-21 用户实测）。

    返回 ``(front_hard, front_slow, left, right, rear_hard, rear_slow)``：
      * ``front_*`` / ``rear_*``：走廊内（``|lat| <= corridor_m``）车头/车尾方向最近距离
      * ``left`` / ``right``：左前 / 右前 30~70° 扇区最近距离（判断"能不能转"）
    """
    q = dict(DEFAULTS if p is None else p)
    corridor = q['corridor_m']
    front_hard = front_slow = math.inf
    rear_hard = rear_slow = math.inf
    left = right = math.inf
    n = len(ranges) if ranges is not None else 0
    for i in range(n):
        r = ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.02 or r > 12.0:
            continue
        a = angle_min + i * angle_increment
        lat = abs(r * math.sin(a))
        lon = r * math.cos(a)
        if lat <= corridor:
            if lon >= 0.0:
                front_hard = min(front_hard, r)
                front_slow = min(front_slow, r)
            else:
                d = -lon
                rear_hard = min(rear_hard, d)
                rear_slow = min(rear_slow, d)
        deg = math.degrees(a)
        if 30.0 <= deg <= 70.0:
            left = min(left, r)
        elif -70.0 <= deg <= -30.0:
            right = min(right, r)
    return front_hard, front_slow, left, right, rear_hard, rear_slow


def decide(person, lost_s: float, clear_hard: float, clear_slow: float,
           left_m: float, right_m: float, p: dict | None = None,
           size_hold: bool = False, rear_hard: float = math.inf,
           rear_slow: float = math.inf, back_hold: bool = False,
           gate_hold: bool = False, gate_center: float = 0.0,
           gate_exit: bool = False, search: bool = False,
           lost_side: float = 0.0, err_dot: float = 0.0) -> tuple:
    """返回 ``(v, w, reason, box_h)``。

    决策顺序（2026-11-21 第三版，用户要求"车头一直追着框、太近就退、退也要避障"）：

    1. 没人 / 丢太久 → 停
    2. **偏离中线太多 → 原地转向对准**（人从车旁边走过时，车头跟着框转，不前冲）
    3. **太近 → 后退**（车尾走廊 0.30 m 内有东西就不退，只原地转）
    4. 到停距 → 停（带迟滞）
    5. 太远 → 只转不追
    6. 车头走廊有障碍 → 停 / 降速

    第四版插入（2026-11-21，用户报"出门框卡在门框上"）：
    * **门前/门内（gate_hold）**：改瞄**门洞中心**、先站正再走、门内限速；
      这一段**不听画面的框**（人走太快时跟着转 → 车角扫地门框）。
    * 车头 0.80 m 内有东西时，把"先对准再走"的门槛从 30% 收紧到 12%（防切内角）。
    """
    q = dict(DEFAULTS)
    if p:
        q.update(p)

    box_h = float(person.get('h', 0.0)) if person is not None else 0.0
    lost_vis = (person is None) or (lost_s > q['lost_stop_s'])

    if lost_vis:
        # A) 门内丢人：人肯定已经先出门了 → **车先把门框走完**（2026-11-21 用户要求）
        # ⚠️ 2026-11-22：这里**不能再要求 gate_hold**。车快出门时前雷达已经看不到
        # "门洞"（门框落在车侧/车后），gate_hold 会先掉；旧写法这时直接落到
        # "no_target → 停"，车就停在门框里，而门线永远退不到车尾后面 → 死锁。
        # 现在只要还在"出门状态（gate_exit）"就继续沿门中心低速往前走，
        # **直到 door_clear（整车越过门线 + 扫掠余量）才允许转身**。
        if gate_exit:
            if clear_hard < q['front_hard_m']:
                return 0.0, 0.0, 'obstacle_stop', box_h
            gw = max(-q['gate_wmax'], min(q['gate_wmax'], q['gate_kp'] * gate_center))
            if abs(gate_center) > q['gate_align_m']:
                return 0.0, gw, 'gate_align', box_h
            return min(q['gate_vmax'], q['vmax']), gw, 'gate_exit', box_h
        # B) 出门口后原地转身找：朝**最后看到人**的那一侧转（右 → ω<0）
        if search:
            side = 1.0 if lost_side > 0 else (-1.0 if lost_side < 0 else 0.0)
            return 0.0, -q['search_wmax'] * side, 'search', box_h
        if person is None:
            return 0.0, 0.0, 'no_target', 0.0
        if lost_s > q['lost_abort_s']:
            return 0.0, 0.0, 'target_gone', box_h
        return 0.0, 0.0, 'target_lost', box_h

    # —— 转向：把目标拉回画面中间（跟随专用增益，比导航快）——
    center_x = person['x'] + person['w'] / 2.0
    err = center_x - 0.5
    # 大幅偏到画面边上 = 要原地转身对准，允许用满 wmax；正常跟随只给 0.42
    w_lim = q['wmax'] if abs(err) > q['face_ex'] else q['steer_wmax']
    w = steer(err, err_dot, q, w_lim)   # PD（含死区/限幅），别再退回纯 P

    # 0) 门前/门内：改瞄门洞中心，别再跟着框左右摆（2026-11-21 第四版）
    #    门框在车身走廊外侧，走廊避障看不见，只靠跟框转向必然擦门框。
    if gate_hold:
        if clear_hard < q['front_hard_m']:
            return 0.0, 0.0, 'obstacle_stop', box_h
        # ⚠️ 符号：雷达 y>0 = 车**左**边（REP-103），门洞中心偏左要**左转**（ω>0）。
        # 画面框是 x 增大向右，所以跟框那条是 -kp·err；这里不能照抄，符号相反。
        gw = max(-q['gate_wmax'], min(q['gate_wmax'], q['gate_kp'] * gate_center))
        if abs(gate_center) > q['gate_align_m']:
            # 还没对准门洞中心 → 原地站正（v=0，转得比跟框慢，不冲出去）
            return 0.0, gw, 'gate_align', box_h
        # 已对准：低速直线穿过去，横向只做很小的修正
        v = min(q['gate_vmax'], q['vmax'])
        if box_h >= q['stop_box_h']:
            return 0.0, gw, 'too_close', box_h   # 门前有人堵着，先别进
        return v, gw, 'gate_pass', box_h

    # 1) 目标偏到画面边上（人从车旁走过）→ 原地转向对准，不前进也不后退
    face_ex = q['face_ex']
    #    车头 0.8 m 内有东西（门框/墙/家具）→ 先对准再走，别一边转一边切内角
    if min(clear_hard, clear_slow) < q['align_first_m']:
        face_ex = min(face_ex, q['align_first_ex'])
    if abs(err) > face_ex:
        return 0.0, w, 'facing', box_h

    # 2) 太近（人贴到车头）→ 后退；车尾有障碍就只能原地转
    if back_hold:
        if rear_hard < q['back_hard_m']:
            # 车尾太近：不许退，只原地转向（用户要求"后面很靠近障碍就原地转"）
            return 0.0, w, 'rear_blocked', box_h
        if rear_slow < q['back_slow_m']:
            span = max(1e-6, q['back_slow_m'] - q['back_hard_m'])
            ratio = max(0.0, min(1.0, (rear_slow - q['back_hard_m']) / span))
            v = -q['back_vmax'] * ratio
        else:
            v = -q['back_vmax']
        return v, w, 'backing', box_h

    # 3) 到停距（迟滞：停过之后要缩到 resume_box_h 才重新走）
    if box_h >= q['stop_box_h'] or (size_hold and box_h > q['resume_box_h']):
        return 0.0, w, 'too_close', box_h

    # 4) 太远 → 只转不追（人在远处，先把车头对准）
    if box_h <= q['far_box_h']:
        return 0.0, w, 'too_far', box_h

    # 5) 车头走廊有障碍 → 停
    if clear_hard < q['front_hard_m']:
        return 0.0, w, 'obstacle_stop', box_h

    # 6) 要往哪边转，那边得有空间
    side = left_m if w > 0 else right_m
    if abs(w) > 0.05 and side < q['side_clear_m']:
        return 0.0, 0.0, 'side_blocked', box_h

    # 7) 速度：按框高在 (stop_box_h, go_box_h) 之间线性插值
    span = max(1e-6, q['stop_box_h'] - q['go_box_h'])
    speed_ratio = min(1.0, max(0.0, (q['stop_box_h'] - box_h) / span))
    v = q['vmax'] * speed_ratio
    # 减速区：车头走廊 0.35~0.60 m 有东西 → 半速（operator 观察到的"门口慢慢挪"）
    if min(clear_hard, clear_slow) < q['front_slow_m']:
        v = min(v, q['vmax'] * 0.65)     # 原来 0.5 → 0.65，减速差别那么狠
        return v, w, 'obstacle_slow', box_h
    return v, w, 'following', box_h


# --------------------------------------------------------------------------
# P2：相机 ↔ 雷达融合（2026-11-24）
# --------------------------------------------------------------------------

def cam_bearing(cx: float, p: dict | None = None) -> float:
    """画面归一化横坐标 ``cx``（0=左、1=右）→ 车头坐标系方位角（rad，左正）。

    符号必须和雷达一致：+ 表示目标在车**左**侧，这样两者能直接比较、也能共用
    ``steer``/``lidar_follow`` 的转向号。画面 x 增大向右 → 方位角为负。
    """
    q = dict(DEFAULTS if p is None else p)
    half = math.radians(float(q['cam_hfov_deg'])) / 2.0
    return -(float(cx) - 0.5) * 2.0 * half


def pick_lidar_track(cands, bearing: float, last_id: int = -1,
                     p: dict | None = None):
    """在雷达轨迹里挑出"相机看到的这个人"。

    ``cands`` 每项需要 ``id`` / ``bearing``（rad，车头系，左正）/ ``r``（m）。

    规则：
      1. 方位差必须在 ``lidar_assoc_deg`` 内（默认 25°）；
      2. 距离在 0.4~4.0 m（更近是车身，更远看不清）；
      3. **上次认领过的 id 优先**（黏住，避免在两腿/家具之间来回跳）；
      4. 否则取方位差最小的。
    返回挑中的条目或 ``None``。
    """
    q = dict(DEFAULTS if p is None else p)
    gate = math.radians(float(q['lidar_assoc_deg']))
    ok = [c for c in (cands or [])
          if 0.4 <= float(c.get('r', 0.0)) <= 4.0
          and abs(float(c.get('bearing', 0.0)) - bearing) <= gate]
    if not ok:
        return None
    for c in ok:
        if int(c.get('id', -1)) == int(last_id):
            return c
    return min(ok, key=lambda c: abs(float(c.get('bearing', 0.0)) - bearing))


def reacquire_track(cands, last_bearing, last_r, p=None, last_id=-1):
    """雷达轨迹**换号/短暂消失**后，按"刚才那个位置"重新认领同一个人。

    为什么需要：`leg_tracker` 的 id 在实流里不稳（腿交叉、两腿合并/分开、
    家具干扰都会让 id 重生）。相机已经看不到人时，id 一换就等于"目标丢了"，
    现场表现就是**雷达跟着跟着突然断开**（2026-11-24 用户报）。

    只接受三个条件都满足的轨迹，避免跟错：
    1. **非 static**（椅子/桌沿不动，会被 leg_tracker 标 static）；
    2. 方位差 ≤ `lidar_reacq_deg`（20°）；
    3. 距离差 ≤ `lidar_reacq_dr`（0.90 m）。

    `last_id` 还在就原样返回（优先黏住）。
    """
    p = p or DEFAULTS
    if last_bearing is None or last_r is None:
        return None
    for c in (cands or []):
        # 同一条（相机点名过的）轨迹：**允许 static** —— 人停下来等车是正常的，
        # 一旦因为"静止"把它踢掉，现场就是"跟着跟着突然断开"。
        if int(c.get('id', -1)) == int(last_id):
            return c
    best, best_cost = None, 1e9
    for c in (cands or []):
        if c.get('static'):
            continue
        db = abs(float(c['bearing']) - float(last_bearing))
        dr = abs(float(c['r']) - float(last_r))
        if db > math.radians(p['lidar_reacq_deg']) or dr > p['lidar_reacq_dr']:
            continue
        cost = dr + 0.3 * db          # 距离更可信：偏差相近时挑近的
        if cost < best_cost:
            best, best_cost = c, cost
    return best


def lidar_follow(bearing: float, rng: float, clear_hard: float,
                 clear_slow: float, left_m: float, right_m: float,
                 p: dict | None = None, close_hold: bool = False) -> tuple:
    """相机丢人后的"雷达续跟"控制律，返回 ``(v, w, reason)``。

    和 ``decide`` 的区别：这里只有**方位 + 距离**，没有框高。
    距离阈值按"人腿"标定：站 1.2 m 停、退到 1.55 m 再走（对标框高 85%/81%）。

    安全约束（任何一条命中都只减速/停，绝不加大速度）：
      * 车头走廊 ``front_hard_m`` 内有东西 → 停；
      * 要转向的那一侧净空不足 → 不转也不走（``side_blocked``）；
      * **绝不后退**（雷达只看得到腿，分辨不出身后是不是人）。
    """
    q = dict(DEFAULTS if p is None else p)
    b = float(bearing)
    # 方位误差 → 角速度（左正，和雷达 y 轴一致；不要抄画框那条的负号）
    if abs(b) < q['lidar_bearing_deadband']:
        w = 0.0
    else:
        w = max(-q['lidar_wmax'], min(q['lidar_wmax'], q['lidar_kp'] * b))

    # 1) 目标在车侧/车后：原地转身对准，不前进（差速车无法侧移）
    if abs(b) > q['lidar_face_rad']:
        w = max(-q['wmax'], min(q['wmax'], q['lidar_kp'] * 2.0 * b))
        return 0.0, w, 'lidar_face'

    # 2) 车头走廊有障碍 → 停（不转也不走，先把障碍弄走）
    if clear_hard < q['front_hard_m']:
        return 0.0, 0.0, 'obstacle_stop'

    # 3) 太近 → 停（带迟滞：停过之后退到 lidar_resume_m 才重新走）
    if rng <= q['lidar_stop_m']:
        return 0.0, w, 'lidar_close'
    if close_hold and rng < q['lidar_resume_m']:
        return 0.0, w, 'lidar_close'

    # 4) 太远 → 只转不追
    if rng >= q['lidar_far_m']:
        return 0.0, w, 'lidar_far'

    # 5) 要往哪边转，那边得有空间
    side = left_m if w > 0 else right_m
    if abs(w) > 0.05 and side < q['side_clear_m']:
        return 0.0, 0.0, 'side_blocked'

    # 6) 距离插值给速度（stop~resume 之间线性，resume 以上满速）
    span = max(1e-6, q['lidar_resume_m'] - q['lidar_stop_m'])
    ratio = max(0.0, min(1.0, (rng - q['lidar_stop_m']) / span))
    v = q['lidar_vmax'] * ratio
    if min(clear_hard, clear_slow) < q['front_slow_m']:
        v = min(v, q['lidar_vmax'] * 0.65)
        return v, w, 'lidar_slow'
    return v, w, 'lidar_follow'


def passage_metrics(ranges, angle_min: float, angle_increment: float,
                    p: dict | None = None):
    """测车前 1.1 m 内有没有"窄通道"（门洞），返回 ``(gap_m, center_m)``，没有则 ``None``。

    为什么需要：门框在车身走廊（±0.22 m）**外面**（横向 0.4~0.6 m），
    走廊避障永远看不见它；跟着人转的时候车角扫过去就顶在门框上（2026-11-21 用户报）。
    这里专门在走廊**外侧**找两侧边界：
      * ``y_left``  = 左边界（y>0）里最靠车轴的那条
      * ``y_right`` = 右边界（y<0）里最靠车轴的那条
      * 净宽 ``gap = y_left - y_right``，中心 ``center = (y_left + y_right)/2``
    只有净宽在 ``gate_min_gap_m ~ gate_wide_m`` 之间才算门洞
    （比车窄 → 不进去；比 1.05 m 宽 → 走廊/大厅，正常跟人）。
    """
    q = dict(DEFAULTS if p is None else p)
    look = q['gate_look_m']
    span = q['gate_half_span_m']
    y_left = math.inf
    y_right = -math.inf
    n = len(ranges) if ranges is not None else 0
    for i in range(n):
        r = ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.02 or r > 12.0:
            continue
        a = angle_min + i * angle_increment
        lon = r * math.cos(a)
        lat = r * math.sin(a)
        if lon < 0.15 or lon > look:
            continue
        if abs(lat) < q['corridor_m'] + 0.02 or abs(lat) > span:
            continue
        if lat > 0.0:
            y_left = min(y_left, lat)
        else:
            y_right = max(y_right, lat)
    if not (math.isfinite(y_left) and math.isfinite(y_right)):
        return None
    gap = y_left - y_right
    if gap < q['gate_min_gap_m'] or gap > q['gate_wide_m']:
        return None
    return gap, (y_left + y_right) / 2.0


def door_metrics(ranges, angle_min: float, angle_increment: float,
                 p: dict | None = None):
    """量"门线"在车身坐标系里的纵向位置，返回 ``(front_lon, rear_lon, gap)`` 或 ``None``。

    **为什么要这个**（2026-11-21 用户报）：出门口时只凭**前雷达**看不到门框就判断
    "已出门"是错的 —— 前雷达装在车头（+0.21 m），它过去时**车尾还在门框里**，
    这时候原地转身必然蹭门框。必须用 **360° `/scan_merged`（前+后雷达）**，
    看门框点退到**车尾后面**才算整车出去。

    做法：在车身两侧横向 ``door_lat_min_m ~ door_lat_max_m``（0.28~0.80 m，正好是
    门框所在）找左右两侧的点，两侧都有、净宽 0.5~1.35 m 才算"门框还在"，
    然后给出：
      * ``front_lon`` —— 最靠车头的门框点（**正 = 还没到车身，负 = 已在车后**）
      * ``rear_lon``  —— 最靠车尾的门框点
    整车出去的条件见 ``door_clear()``。
    """
    q = dict(DEFAULTS if p is None else p)
    lat_lo = q['door_lat_min_m']
    lat_hi = q['door_lat_max_m']
    lon_span = q['gate_look_m'] + 0.2
    lons = []
    y_left, y_right = math.inf, -math.inf
    n = len(ranges) if ranges is not None else 0
    for i in range(n):
        r = ranges[i]
        if r is None or not math.isfinite(r) or r <= 0.02 or r > 12.0:
            continue
        a = angle_min + i * angle_increment
        lon = r * math.cos(a)
        lat = r * math.sin(a)
        if abs(lon) > lon_span:
            continue
        alat = abs(lat)
        if alat < lat_lo or alat > lat_hi:
            continue
        lons.append(lon)
        if lat > 0:
            y_left = min(y_left, lat)
        else:
            y_right = max(y_right, lat)
    if not lons or not (math.isfinite(y_left) and math.isfinite(y_right)):
        return None
    gap = y_left - y_right
    if gap < 0.50 or gap > 1.35:       # 不是"一扇门"，是走廊/开阔地
        return None
    return max(lons), min(lons), gap


def door_swept_radius(p: dict | None = None) -> float:
    """原地转身时车身扫过的圆半径（车尾角点离 base_link 最远）。"""
    q = dict(DEFAULTS if p is None else p)
    return math.hypot(q['body_rear_m'], q['body_half_width_m'])


def door_clear(front_lon, p: dict | None = None) -> bool:
    """门线是否已经**整车**开出门框（含原地转身的扫掠余量）。

    ⚠️ 2026-11-22 用户第二次报「出门时车没完全出来就转向」：旧判据是
    ``front_lon < -(body_rear_m + 0.06) = -0.29``，也就是"门线退到车尾保险杠
    后面 6 cm"。但车是**原地转**的：车尾左右角点离 base_link 的距离是
    ``hypot(0.23, 0.215) = 0.315 m``，转到 45° 时后角点会甩到 lon = -0.315，
    **比 -0.29 还靠后 2.5 cm** → 就是"看着出去了、一转就蹭门框"。

    现在改用**扫掠半径** ``R = hypot(body_rear_m, body_half_width_m)``，
    门线必须退到 ``-R - door_clear_margin_m`` 之后（默认 -(0.315+0.10) = **-0.415 m**，
    即车尾后面约 18 cm）才允许转身。
    """
    q = dict(DEFAULTS if p is None else p)
    if front_lon is None or not math.isfinite(front_lon):
        return True
    return front_lon < -(door_swept_radius(q) + q['door_clear_margin_m'])


def steer(err: float, err_dot: float = 0.0, p: dict | None = None,
          w_lim: float | None = None) -> float:
    """把"画面偏差"换成角速度：**PD**（2026-11-21 晚修左右摆）。

    旧写法只有 P：``w = -kp·e``，``kp 3.5``。底盘角速度响应有 0.2~0.3 s 滞后，
    纯 P 高增益必然进极限环 —— 用户看到的就是"一直左转右转"。
    现在加阻尼项（``err_dot`` = d(框中心偏差)/dt）+ 死区 + 跟随专用限幅：

        w = -(kp·e + kd·err_dot)

    阻尼为什么是这个符号：车往右转时框中心偏差在变小 → ``err_dot < 0`` →
    ``-kd·err_dot > 0`` 正好抵消一部分右转，也就是"快到位了提前收力"。
    参考实现：`Pramod-Gurav/ros2-rgbd-person-follower`（angular PID，kp 0.004/px、
    30 px 死区、EMA 0.2、输出 ±1.0 rad/s），我们的满幅等效增益比它更低更稳。
    """
    q = dict(DEFAULTS if p is None else p)
    db = q['yaw_deadband']
    if abs(err) <= db:
        return 0.0
    e = err - math.copysign(db, err)          # 死区外重新起算，避免 0 处跳变
    w_p = q['yaw_kp'] * e
    # ⚠️ 阻尼只许"收力"，不许把车拽反方向：上限 -70%~+30% 的 P 项。
    #    不夹的话 err_dot 一大，`-kd·err_dot` 会反号（实测 err=+0.10、dot=-0.5
    #    会输出 +0.35 = 往反方向转，跟着人直线走照样左右摆）。
    w_d = q['yaw_kd'] * float(err_dot or 0.0)
    w_d = max(-0.70 * abs(w_p), min(0.30 * abs(w_p), w_d))
    w = -(w_p + w_d)
    lim = min(q['wmax'] if w_lim is None else w_lim, q['wmax'])
    return max(-lim, min(lim, w))


class OscGuard:
    """"左右摆"自动阻尼（2026-11-21 用户报"车一直左转右转"）。

    纯几何推导算不出真车的极限环（底盘占空比爬升 `DUTY_RAMP_PER_S 0.5`、轮速环、
    万向轮摩擦都参与），所以这里直接盯**观测量**：短窗口内 ω 反复变号 = 正在振。
    一旦振起来就按比例收转向增益（最少收到 35%），稳下来再慢慢恢复。
    等价于"自动降 P"，但不依赖对模型的假设。

    用法：``g = guard.update(w_raw, dt)``，输出 ``w_raw * g``。
    """

    def __init__(self, min_w: float = 0.08, window_s: float = 2.0,
                 flip_limit: int = 3, floor: float = 0.35,
                 decay: float = 0.6, recover_per_s: float = 0.12) -> None:
        self.min_w = min_w
        self.window_s = window_s
        self.flip_limit = flip_limit
        self.floor = floor
        self.decay = decay
        self.recover_per_s = recover_per_s
        self.gain = 1.0
        self._t = 0.0
        self._last_sign = 0
        self._flips: list = []

    def reset(self) -> None:
        self.gain = 1.0
        self._last_sign = 0
        self._flips.clear()

    def update(self, w: float, dt: float) -> float:
        self._t += max(0.0, dt)
        self._flips = [t for t in self._flips if self._t - t <= self.window_s]
        sign = 0 if abs(w) < self.min_w else (1 if w > 0.0 else -1)
        if sign:
            if self._last_sign and sign != self._last_sign:
                self._flips.append(self._t)
            self._last_sign = sign
        if len(self._flips) >= self.flip_limit:
            self.gain = max(self.floor, self.gain * self.decay)
            self._flips.clear()
            self._last_sign = 0
        else:
            self.gain = min(1.0, self.gain + self.recover_per_s * max(0.0, dt))
        return self.gain


def limit_pair(v: float, w: float, p: dict | None = None) -> tuple:
    """把 (v, ω) 收到底盘能接受的范围（2026-11-21 实机踩坑）。

    两道硬限（`base_bridge.manual_velocity`）：
      1. ``|ω| ≤ 0.75`` —— 超了整帧被丢，车一动不动；
      2. 单轮 ``|v| + |ω|·track/2 ≤ manual_max_speed_mps``（0.30）—— 超了同样被丢。

    这里保 ω（转向是跟随的命门）、按剩余预算收 v，曲率略有变化但不会被丢帧。
    """
    q = dict(DEFAULTS if p is None else p)
    w = max(-q['wmax'], min(q['wmax'], w))
    w = max(-0.74, min(0.74, w))
    room = q['max_wheel_mps'] - abs(w) * q['track_m'] / 2.0
    room = max(0.0, room)
    if abs(v) > room:
        v = math.copysign(room, v)
    return v, w
