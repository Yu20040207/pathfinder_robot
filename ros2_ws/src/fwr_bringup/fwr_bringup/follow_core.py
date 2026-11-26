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
    'stop_box_h': 0.80,   # 框高 ≥ 80% → 停（2026-11-26 用户要求 85 → 80）
    'resume_box_h': 0.76, # 停过之后框缩到 76% 就重新走（迟滞 4%，留 1% 余量：
                          # "框 80% 时必须在动"，卡在 0.80 正好是边界会犹豫）
    # ⚠️ 这个值越大，跟得越快：0.62 时"框 0.65~0.70"（最常见的跟随距离）只有
    # 0.19→0.10 m/s，用户 2026-11-21 反馈"太慢"。改成 0.70：框 ≤0.70 全速，
    # 0.70~0.75 才减速，减速窗口只剩 5%（靠 v_accel 0.6 平滑，不会顿）。
    'go_box_h': 0.72,     # 框 ≤72% 全速；72~80% 之间按比例减速（跟 stop_box_h 联动）
    'far_box_h': 0.30,    # 框高 ≤ 30%（≈6 m 外）→ 不追，只转过去看
    # —— 速度 / 转向（跟随专用，改完不影响导航）——
    # 上限由底盘硬限决定：manual_max_speed_mps 0.30 − |ω|·track/2 ⇒ 直行最多 0.29。
    # 2026-11-21 用户再说"太慢"：0.25 → 0.29（在不改底盘安全参数的前提下到顶）。
    'vmax': 0.38,         # 2026-11-26 固件协议 300→400 mm/s 后：0.29 → 0.38
    # ⚠️ 必须 < 0.75：base_bridge.manual_velocity() 硬限 |ω| ≤ 0.75，超了整帧被丢
    # （日志 Rejected out-of-range manual velocity）→ 页面显示"正在转向"但车一动不动。
    # 第二版曾设 0.9，实机 facing/转弯时全被拒（2026-11-21 实测确认）。
    'wmax': 0.74,        # 原地对准/雷达对脸的 ω 上限（底盘硬限 0.75）
    # 2026-11-21 晚（用户报"直线跟随左右摆"）：纯 P + 3.5 增益在慢底盘上必然极限环。
    # 改成 PD：kp 3.5 → 2.4，加 kd 阻尼，跟随阶段 ω 上限单独夹 0.42。
    # 参考开源 `Pramod-Gurav/ros2-rgbd-person-follower`（angular PID + EMA + 30 px 死区，
    # kp 0.004/px 折合满幅约 1.3 rad/s，比我们原来的 3.5 温和得多）。
    'yaw_kp': 2.4,        # 2.2 → 3.5，同样为跟得上人横向走动
    'yaw_kd': 0.9,        # 阻尼：d(框中心偏差)/dt 的增益（rad/s 每 1/s 偏差变化）
    'steer_wmax': 0.74,   # 跟随时 ω 上限：0.42 → 0.74（翻倍要求，0.75 是底盘硬限）
                          # ⚠️ 只在跟随模式生效，手动/导航不读这些参数
    'yaw_deadband': 0.06,   # 画面中间 ±5% 不转（检测框每帧会抖，3% 太敏感）
    # —— 走廊避障（车身半宽 0.215，留 5 mm）——
    # 单轮速度包络：base_bridge manual_max_speed_mps=0.30，超了同样整帧被丢。
    # 留 1 cm/s 余量，命令按 |v| + |ω|·track/2 ≤ max_wheel 收（保曲率，收线速度）。
    'max_wheel_mps': 0.38,   # 单轮包络上限，比 manual_max_speed_mps 0.40 留 2 cm/s 余量
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
    'search_wmax': 0.74,
    'back_box_h': 0.92,     # 框高 ≥ 92% → 人贴脸了，往后退
    'back_release_box_h': 0.80,   # 退到 80% 才停（跟 stop_box_h 对齐）
    # 后退速度（比前进慢）。2026-11-26 用户要求 0.12 → **0.22**（嫌退得太慢）：
    # 与前进 0.38 一起算单轮包络 0.22 + 0.42×0.317/2 = 0.287 < max_wheel_mps 0.29，
    # 仍不会被底盘整帧丢弃；车尾 0.60 m 内降速、0.30 m 内禁止后退的保护不动。
    'back_vmax': 0.22,
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
    'lidar_reacq_deg': 15.0,    # 重新认领：方位差上限（°）——放宽到 28° 会认到旁边的东西
    'lidar_reacq_dr': 0.60,     # 重新认领：距离差上限（m）——1.2 m 太宽，容易换成别的目标
    'lidar_max_speed': 2.0,     # 轨迹速度超过这个就**不认**（推车/甩动的门/被拖的椅子）
    'lidar_search_s': 4.0,      # 雷达短暂丢失后，原地按最后方位找人的时长（s）
    'lidar_hold_max_s': 30.0,   # 相机完全看不到的最长接管时间（防止跟着残影走一整天）
    # 2026-11-26 用户报"我已经在旁边走过去了，雷达跟踪还往前跟踪中"：
    # 相机一丢人，雷达只认 id 就一路往前开，最长能开 `lidar_hold_max_s`。
    # 雷达分辨不出"这个人"和"前面那个人/那个货"，所以规定：
    # **相机确认过之后最多盲开 `lidar_blind_drive_s` 秒**，超时只许原地转向找人，
    # 不许再往前开。相机一看到人（背书刷新）立刻恢复。
    'lidar_blind_drive_s': 2.5,
    'lidar_adopt_m': 0.80,      # 目标丢了按"预测位置"重新认领：基础半径（m）
    'lidar_adopt_mps': 0.60,    # ……每过 1 s 再放宽多少米（人在走）
    'lidar_adopt_vmin': 0.12,   # 认领时要求轨迹**正在走动**（m/s），家具达不到
    'lidar_pred_max_s': 2.0,    # 匀速预测最多外推多久（超过就不猜了）
    'lidar_static_drop_s': 2.0, # 续跟时目标"停了且没相机背书"多久就放弃（家具没有动静）
    'lidar_fresh_s': 0.5,       # 雷达轨迹多久没更新就算过期（10 Hz 扫描，0.5 s 够宽）
    'lidar_assoc_deg': 18.0,    # 相机看到人时，用它对准雷达轨迹（方位差上限）
    'lidar_back_max_s': 2.0,    # 轨迹"相机背书"的有效期（超时就不许认领/续认）
    'cam_hfov_deg': 70.0,       # 相机水平视场（HP60C 实测约 70°），框中心 → 方位角
    'lidar_stop_m': 1.28,       # 雷达目标近到这个距离 → 停（对标"框 80%"，原来是 1.20/85%）
    'lidar_resume_m': 1.55,     # 停过之后退到这么远才重新走（迟滞 35 cm）
    'lidar_far_m': 3.00,        # 超过这个距离只转不追（看不清是腿还是家具）
    'lidar_face_rad': 0.50,     # 目标偏离车头超过 29° → 原地转向对准，不前进
    # 2026-11-26 用户报"雷达接管时左转右转对准幅度大，刷新后重新跟随就稳"：
    # 定位到 `lidar_face` 支路是**纯 P 且增益 ×2**（3.6·b），只要进 face 就顶到 0.74 rad/s
    # 满速转；20 Hz + w_accel 3.0 下刹不住 → 冲过头 → 反着修 → 左右摆。
    # 相机那条路早有 PD + 滞回，雷达这条一直没做。现在补三件：
    #   ① face 增益 2.0 → 1.15，且 face 期间限速到 lidar_face_wmax；
    #   ② 阻尼项 lidar_kd·(方位变化率)：方位在收（趋近 0）时提前松力；
    #   ③ 滞回：>lidar_face_rad 进 face，<lidar_face_exit_rad 才退出（防阈值抖动）。
    'lidar_face_exit_rad': 0.38,  # 退出 face 的滞回阈值（<进门槛，防来回切）
    'lidar_face_gain': 1.15,    # face 时的 P 增益倍数（原来硬编码 2.0）
    'lidar_face_wmax': 0.55,    # face 期间转向上限（满速 0.74 会冲过头）
    'lidar_bearing_deadband': 0.05,   # ±3° 不转（雷达有抖动）
    'lidar_kp': 1.8,            # 方位误差 → 角速度
    'lidar_kd': 0.25,           # 方位变化率 → 阻尼（按闭合方向减力）
    'lidar_bearing_smooth': 0.5,      # 方位 EMA（雷达方位逐帧抖 2~5°）
    'lidar_flip_reject_rad': 0.60,    # 换轨迹导致方位突变超过 34° → 先不信，等确认
    'lidar_flip_confirm_s': 0.30,     # 突变要持续这么久才认
    'lidar_wmax': 0.74,         # 雷达接管时的转向上限（与跟框 steer_wmax 一致）
    'lidar_vmax': 0.20,         # 雷达接管时的前进上限（没相机辅助，仍比跟随慢一档）
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


def _lidar_backed(c: dict, p: dict) -> bool:
    """这条雷达轨迹"此刻还算不算相机确认过的人"。

    `cam` 是 leg_tracker 发的**新鲜背书**（1 s 内被相机确认）；`cam_age_s` 用来给
    "刚丢一瞬间"留一点余量（人站住不动、相机刚丢 1~2 s）。其余一律不认，
    否则货箱/椅子被永久当成的人（2026-11-26 现场）。
    """
    if c.get('cam'):
        return True
    age = c.get('cam_age_s')
    lim = float(p.get('lidar_back_max_s', DEFAULTS['lidar_back_max_s']))
    return age is not None and float(age) <= lim


def lidar_backed(track: dict, p: dict | None = None) -> bool:
    """`_lidar_backed` 的公开版：这条轨迹"此刻还算不算相机确认过的人"。"""
    return _lidar_backed(track or {}, dict(DEFAULTS if p is None else p))


def lidar_drive_allowed(blind_age_s: float, p: dict | None = None) -> bool:
    """相机丢人之后，雷达还能不能**往前开**。

    2026-11-26 用户报"我已经在旁边走过去了，雷达跟踪还往前跟踪中"：
    雷达只认 id，分辨不出"这个人"和"前面那个人/推车"，一路往前开最长 30 s。
    现在只允许盲开 `lidar_blind_drive_s`（2.5 s），超时只能原地转向找人；
    相机一旦重新看到人（背书刷新、`blind_age` 归零）立即恢复前进。
    """
    q = dict(DEFAULTS if p is None else p)
    return float(blind_age_s) <= float(q['lidar_blind_drive_s'])


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
          if 0.6 <= float(c.get('r', 0.0)) <= 3.5
          and abs(float(c.get('bearing', 0.0)) - bearing) <= gate
          and float(c.get('v', 0.0)) <= float(q['lidar_max_speed'])
          and not c.get('static')
          and _lidar_backed(c, q)]
    if not ok:
        return None
    # **只认相机背书过的轨迹**：以前"没有背书的也退而求其次"，结果货箱/推车被认领后
    # 就一路跟着它（2026-11-26 现场截图：画面里只有货箱，提示"雷达接管中，正朝你过去"）。
    pool = ok
    for c in pool:
        if int(c.get('id', -1)) == int(last_id):
            return c
    return min(pool, key=lambda c: abs(float(c.get('bearing', 0.0)) - bearing))


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
            continue                      # 静止的一律不续认（人站住等车会靠"原地找人"扫回来）
        if not _lidar_backed(c, p):
            continue                      # 没被相机确认过的轨迹永远不许认领
        if float(c.get('v', 0.0)) > float(p['lidar_max_speed']):
            continue
        db = abs(float(c['bearing']) - float(last_bearing))
        dr = abs(float(c['r']) - float(last_r))
        if db > math.radians(p['lidar_reacq_deg']) or dr > p['lidar_reacq_dr']:
            continue
        cost = dr + 0.3 * db          # 距离更可信：偏差相近时挑近的
        if cost < best_cost:
            best, best_cost = c, cost
    return best


def predict_person(mem: dict, now: float, p: dict | None = None) -> tuple:
    """把"人最后在 odom 系的位置 + 速度"按匀速外推到 ``now``。

    `mem` = ``{'x','y','vx','vy','at'}``（odom 系）。外推上限 `lidar_pred_max_s`：
    人站在那儿不动时速度≈0，外推等于原地；人一直走，最多外推 2 s（3 m 级别）就该找到新轨迹了。
    """
    q = DEFAULTS if p is None else p
    if not mem:
        return (0.0, 0.0)
    dt = max(0.0, min(float(now) - float(mem.get('at') or 0.0),
                      float(q['lidar_pred_max_s'])))
    return (float(mem['x']) + float(mem.get('vx') or 0.0) * dt,
            float(mem['y']) + float(mem.get('vy') or 0.0) * dt)


def adopt_moving_track(cands, pred, elapsed: float, p: dict | None = None):
    """相机看不到人、原轨迹又换号时，按"预测位置 + 正在走动"认领一条新轨迹。

    为什么需要：`pick_lidar_track` 只认"相机新鲜背书"，人绕到车侧/车后就再也认不回来
    —— 现场表现就是"跟丢之后我还在旁边走，它也认不出我"（2026-11-26 用户报）。

    安全边界（缺一不可）：
      * 必须是**在走动**的轨迹（`lidar_adopt_vmin ≤ v ≤ lidar_max_speed`）—— 椅子/货箱不动；
      * 必须落在**预测位置**附近（半径 `lidar_adopt_m`，每过 1 s 再放宽 `lidar_adopt_mps`）；
      * 静止轨迹、超速轨迹一律不要；
      * 相机背书过的优先（同样的距离，选那条）。
    """
    q = DEFAULTS if p is None else p
    gate = float(q['lidar_adopt_m']) + float(q['lidar_adopt_mps']) * max(0.0, float(elapsed))
    best, best_cost = None, 1e9
    for c in (cands or []):
        if c.get('static'):
            continue
        v = float(c.get('v', 0.0))
        if not (float(q['lidar_adopt_vmin']) <= v <= float(q['lidar_max_speed'])):
            continue
        d = math.hypot(float(c['x']) - float(pred[0]), float(c['y']) - float(pred[1]))
        if d > gate:
            continue
        cost = d - (0.35 if c.get('cam') else 0.0)
        if cost < best_cost:
            best, best_cost = c, cost
    return best


def lidar_search_cmd(bearing: float, p: dict | None = None):
    """雷达轨迹**短暂消失**（换号/合并/被挡）时，原地按最后方位慢慢找人。

    为什么要有它：相机已经看不到人、雷达那条腿轨迹又刚好断 0.3~1 s 时，
    旧逻辑直接 `target_lost` → 停车。现场感受就是"雷达跟一会儿突然丢"。
    这里保持**原地只转**（v=0，绝不前进），扫到轨迹后正常接管。
    """
    q = dict(DEFAULTS if p is None else p)
    if bearing is None:
        return 0.0, 0.0, 'lidar_search'
    err = math.atan2(math.sin(float(bearing)), math.cos(float(bearing)))
    if abs(err) <= abs(float(q['lidar_bearing_deadband'])):
        return 0.0, 0.0, 'lidar_search'
    w = max(-float(q['lidar_wmax']), min(float(q['lidar_wmax']),
                                        float(q['lidar_kp']) * err))
    return 0.0, w, 'lidar_search'


def bearing_ema(prev, raw: float, dt: float, alpha: float) -> tuple:
    """雷达方位 EMA + 变化率（rad/s）。返回 ``(smoothed, rate)``。

    雷达方位逐帧抖 2~5°，裸值直接进 PD 就是"抖着转"；变化率用于阻尼项
    （车朝目标转时方位在减小 → rate<0 → 提前松力，见 `_lidar_pd`）。
    """
    raw = float(raw)
    if prev is None:
        return raw, 0.0
    a = max(0.0, min(1.0, float(alpha)))
    smoothed = a * raw + (1.0 - a) * float(prev)
    rate = 0.0
    if dt > 1e-3:
        rate = max(-6.0, min(6.0, (smoothed - float(prev)) / float(dt)))
    return smoothed, rate


def flip_confirm(last_ok, flip, b: float, tid: int, now: float,
                 reject_rad: float, confirm_s: float) -> tuple:
    """换轨迹导致的方位突变要不要信。返回 ``(accept, last_ok, flip)``。

    现场（2026-11-26 日志）：轨迹号在 #34 / #49 / #95 / #112 … 之间跳，
    方位 +25° / -38° / +60° 跟着跳 —— 两条腿/两个人抢 id 时车就"左右各转一炮"。
    规则：方位跳变超过 `reject_rad` 且轨迹号也变了 → 先不信；
    同一个候选连续一致超过 `confirm_s` 才认（真换了目标也能跟上，只是慢 0.3 s）。
    """
    b = float(b)
    tid = int(tid)
    if last_ok is None:
        return True, (b, tid), None
    ok_b, ok_id = last_ok
    if abs(b - ok_b) > float(reject_rad) and tid != int(ok_id):
        if (flip is not None and abs(b - flip[0]) <= float(reject_rad)
                and tid == int(flip[1])):
            if now - flip[2] >= float(confirm_s):
                return True, (b, tid), None
            return False, last_ok, flip
        return False, last_ok, (b, tid, now)
    return True, (b, tid), None


def _lidar_pd(err: float, rate: float, kp: float, kd: float) -> float:
    """方位误差的 PD 输出（左正）。阻尼项**只许收力**，不许把转向反号。

    雷达方位是"目标相对车头"的角度：车朝目标转时方位在**减小**（rate<0）。
    所以 `kd*rate` 在趋近阶段是负向（提前松力），不会把车拽反 —— 但为了绝对安全，
    阻尼量夹在 `±0.5·|kp·err|` 以内，最多抵消一半 P。
    """
    p_term = kp * float(err)
    d_term = kd * float(rate)
    limit = 0.5 * abs(p_term)
    if limit > 0.0:
        d_term = max(-limit, min(limit, d_term))
    return p_term + d_term


def lidar_follow(bearing: float, rng: float, clear_hard: float,
                 clear_slow: float, left_m: float, right_m: float,
                 p: dict | None = None, close_hold: bool = False,
                 bearing_rate: float = 0.0,
                 face_hold: bool = False) -> tuple:
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
    rate = float(bearing_rate)
    # 方位误差 → 角速度（左正，和雷达 y 轴一致；不要抄画框那条的负号）
    if abs(b) < q['lidar_bearing_deadband']:
        w = 0.0
    else:
        w = max(-q['lidar_wmax'], min(q['lidar_wmax'],
                                      _lidar_pd(b, rate, q['lidar_kp'],
                                                q['lidar_kd'])))

    # 1) 目标在车侧/车后：原地转身对准，不前进（差速车无法侧移）
    #    滞回：已经在对准时用更低的退出阈值，避免在 29° 附近来回切模式。
    face_gate = q['lidar_face_exit_rad'] if face_hold else q['lidar_face_rad']
    if abs(b) > face_gate:
        w = max(-q['lidar_face_wmax'], min(q['lidar_face_wmax'],
                _lidar_pd(b, rate, q['lidar_kp'] * q['lidar_face_gain'],
                          q['lidar_kd'])))
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


def rate_damp(w: float, err: float, w_meas: float, kd: float = 0.9,
              window: float = 0.25, wmax: float = 0.74) -> float:
    """中线圈里用**实测转速**做阻尼，专治"对准时左右摆"。

    为什么不能用帧间误差导数：框中心的变化率比真实转动**晚 1~2 帧**（相机 15 fps +
    检测 12 Hz + EMA），而底盘占空比爬升（固件 `DUTY_RAMP_PER_S 0.5`）又让车在
    "命令归零之后"继续转 0.5~1.2 s。于是"看到对准了 → 收力"永远迟一步 →
    过冲 → 反着修 → 来回摆（2026-11-26 用户反复报的现象）。

    实测转速（`/odom` 的 `twist.angular.z`）**没有这个延迟**，直接拿它做
    "反向制动"：车已经在左转，就先把指令往右压一点。只在 `|err| < window`
    的中线圈里生效（大误差还要满速追，别把掉头/大角度对准变慢），
    越靠近中线阻尼越强（`1 - |err|/window`）。
    """
    if abs(err) >= window or kd <= 0.0:
        return w
    gain = 1.0 - abs(err) / window
    out = w - kd * gain * float(w_meas)
    return max(-wmax, min(wmax, out))


def reacq_cap(w: float, err: float, active: bool,
              soft_wmax: float = 0.40, soft_ex: float = 0.45) -> float:
    """重新认领目标后的"软对准"：框还在画面边缘时不许提前收力。

    2026-11-26 用户报：人从旁边绕到车后，车掉头重新看到人之后**对准时左右摆**。
    原因之一是刚认回来就在中线附近满速修：12 Hz 检测 + 83 ms EMA + `w_accel 3.0`
    让它每次都比目标多转一点，来回打。这里在"重新看见人后的 ``reacq_soft_s`` 内、
    误差已经不大（``|err| < soft_ex``）"时把 ω 限到 ``soft_wmax``，
    边缘大误差仍按原来的限幅去追（掉头不能变慢）。
    """
    if not active:
        return w
    if abs(w) <= soft_wmax or abs(err) >= soft_ex:
        return w
    return math.copysign(soft_wmax, w)


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
