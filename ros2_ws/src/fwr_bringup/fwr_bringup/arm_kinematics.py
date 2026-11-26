"""六舵机机械臂运动学（纯函数，便于单测；不含 ROS）。

为什么单独抽出来
----------------
握手的第一步是"这只臂到底够不够得着人的手"。这个问题**完全是几何**：给定臂长和
安装高度，可达空间就定了，不需要硬件、不需要 ROS、不需要相机。先把它算清楚，
再决定要不要加滑台或改安装位，比先写控制代码再发现够不着便宜得多。

坐标系（**只有一个，别搞混**）
------------------------------
所有公开函数都在 **地面坐标系** 里算：x 朝车头、y 朝车左、z 从地面往上，单位米。
臂的内部数学用"相对肩关节"的偏移，进函数时减、出函数时加，只在本模块内部发生。

构型假设（**拿到实物后必须用实测覆盖**）
----------------------------------------
6 个 ST3215 总线舵机，典型串接构型（5 关节 + 1 夹爪）：

    J1 底座偏航（绕 z）
    J2 肩俯仰
    J3 肘俯仰
    J4 腕俯仰
    J5 腕滚转（不影响位置，只影响夹爪朝向）
    J6 夹爪开合（不参与位姿链）

俯仰链的绝对仰角：θ2 = q2、θ3 = q2+q3、θ4 = q2+q3+q4，
即 ``q3/q4`` 是**相对上一节的增量**，0 = 水平前伸、向上为正。

关键量
------
* ``reach_m = l_upper + l_fore + l_grip`` —— 肩到夹爪中心最远
* ``max_hand_height_m = shoulder_height_m + reach_m`` —— 举手能碰到的最高点
  用户给的"手到地 1.25 m"与"臂 25 cm"必须满足这条等式，否则参数有矛盾，有单测钉着。
* 但**夹爪水平时可达范围会缩小**：腕必须落在半径 0.18 m 的球内，夹爪再横伸 4 cm。
  所以"水平伸到人手里"的水平够程是 ``sqrt(0.18² − dz²) + 0.04``，不是 0.22。
  这条容易算错，``horizontal_reach_m`` 专门算它。

为什么用解析解而不是 MoveIt
---------------------------
"末端位置 + 腕俯仰角"这一层是闭式可解的（两连杆 + 腕延伸），单次 ~20 μs，
比数值迭代快两三个数量级，也不需要 1~2 GB 内存。这个臂的自由度本来就做不了任意
姿态，上重型框架纯浪费。

不可达原因必须分开报
--------------------
上层状态机要靠它决定"谁动"：

* ``TOO_FAR``     车再靠近一点就能解决（底盘的事）
* ``TOO_CLOSE``   人太近，臂肘折不回来（人的事）
* ``ANGLE_LIMIT`` 位置可达但关节超出结构限位（换肘向/改腕角）
* ``ABOVE_REACH`` 超出举手高度（折叠机构的事）
* ``BEHIND``      目标在臂正后方（底座转不过去）
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

OK = "ok"
TOO_FAR = "too_far"
TOO_CLOSE = "too_close"
ANGLE_LIMIT = "angle_limit"
ABOVE_REACH = "above_reach"
BELOW_REACH = "below_reach"
BEHIND = "behind"


@dataclass(frozen=True)
class ArmGeometry:
    """臂的几何参数（米 / 弧度）。默认值按"25 cm 臂 + 手最高 1.25 m"反推。"""

    #: 肩关节（J2）离地高度 = 车顶安装面高度 + l_base
    shoulder_height_m: float = 1.03
    l_base: float = 0.030  # J1 轴 → J2 轴，只用于把安装面高度换算成肩高
    l_upper: float = 0.090  # J2 → J3
    l_fore: float = 0.090  # J3 → J4
    l_grip: float = 0.040  # J4 → 夹爪中心

    # 关节结构限位（弧度）。ST3215 本身 0~4095 步（0.088°/步），真正卡住的是结构件。
    q1_min: float = math.radians(-100)
    q1_max: float = math.radians(100)
    q2_min: float = math.radians(-60)
    q2_max: float = math.radians(120)
    q3_min: float = math.radians(-140)
    q3_max: float = math.radians(140)
    q4_min: float = math.radians(-100)
    q4_max: float = math.radians(100)

    @property
    def reach_m(self) -> float:
        return self.l_upper + self.l_fore + self.l_grip

    @property
    def max_hand_height_m(self) -> float:
        """竖直举手能碰到的最高点（离地）。"""
        return self.shoulder_height_m + self.reach_m

    @property
    def mount_height_m(self) -> float:
        """车顶安装面离地高度。"""
        return self.shoulder_height_m - self.l_base

    @property
    def min_wrist_dist_m(self) -> float:
        """肘折到限位时，腕到肩的最近距离。小于它就真的收不回来了。"""
        q3_abs = max(abs(self.q3_min), abs(self.q3_max))
        v = (self.l_upper ** 2 + self.l_fore ** 2
             + 2.0 * self.l_upper * self.l_fore * math.cos(q3_abs))
        return math.sqrt(max(v, 0.0))

    def limits(self, joint: int) -> tuple[float, float]:
        return {
            1: (self.q1_min, self.q1_max),
            2: (self.q2_min, self.q2_max),
            3: (self.q3_min, self.q3_max),
            4: (self.q4_min, self.q4_max),
        }[joint]


DEFAULT_GEOMETRY = ArmGeometry()


def _within(joints, geom: ArmGeometry) -> bool:
    for i, q in enumerate(joints[:4], start=1):
        lo, hi = geom.limits(i)
        if q < lo - 1e-9 or q > hi + 1e-9:
            return False
    return True


def forward(joints, geom: ArmGeometry = DEFAULT_GEOMETRY):
    """正解：关节角 → ``(x, y, z)``，地面坐标系，米。``joints`` 取 ``(q1, q2, q3, q4)``。"""
    q1, q2, q3, q4 = [float(v) for v in joints[:4]]
    t2 = q2
    t3 = q2 + q3
    t4 = q2 + q3 + q4

    r = (geom.l_upper * math.cos(t2)
         + geom.l_fore * math.cos(t3)
         + geom.l_grip * math.cos(t4))
    z = (geom.shoulder_height_m
         + geom.l_upper * math.sin(t2)
         + geom.l_fore * math.sin(t3)
         + geom.l_grip * math.sin(t4))
    return (r * math.cos(q1), r * math.sin(q1), z)


def inverse(target, gripper_pitch: float = 0.0,
            geom: ArmGeometry = DEFAULT_GEOMETRY, elbow_up: bool = False):
    """逆解：目标点（地面系）+ 夹爪**绝对**仰角 → ``(joints, reason)``。

    失败时 ``joints is None``，``reason`` 说明是哪一类不可达，上层据此决定策略。

    为什么先减掉夹爪那一段
    ----------------------
    夹爪中心不是腕关节。给定夹爪仰角 θ4，腕关节 = 目标 − l_grip·(cos θ4, sin θ4)，
    问题就退化成正正的两连杆 IK。反过来（先解两连杆再补夹爪）会得到不一致的解，
    正解回代对不上 —— 这个坑已经踩过一次。
    """
    x, y, z = [float(v) for v in target[:3]]
    r = math.hypot(x, y)

    if z > geom.shoulder_height_m + geom.reach_m + 1e-9:
        return (None, ABOVE_REACH)
    # 臂可以朝下伸，所以下限不是安装面，而是"安装面往下再走满一个臂长"
    if z < geom.mount_height_m - geom.reach_m - 1e-9:
        return (None, BELOW_REACH)

    q1 = math.atan2(y, x)
    lo1, hi1 = geom.limits(1)
    if q1 < lo1 - 1e-9 or q1 > hi1 + 1e-9:
        return (None, BEHIND)

    t4 = float(gripper_pitch)
    rw = r - geom.l_grip * math.cos(t4)
    zw = (z - geom.shoulder_height_m) - geom.l_grip * math.sin(t4)

    dist = math.hypot(rw, zw)
    two_link_max = geom.l_upper + geom.l_fore
    if dist > two_link_max + 1e-9:
        return (None, TOO_FAR)
    if dist < geom.min_wrist_dist_m - 1e-9:
        return (None, TOO_CLOSE)

    cos_q3 = ((dist ** 2 - geom.l_upper ** 2 - geom.l_fore ** 2)
              / (2.0 * geom.l_upper * geom.l_fore))
    q3 = math.acos(max(-1.0, min(1.0, cos_q3)))
    if not elbow_up:
        q3 = -q3

    q2 = math.atan2(zw, rw) - math.atan2(
        geom.l_fore * math.sin(q3), geom.l_upper + geom.l_fore * math.cos(q3))
    q4 = t4 - (q2 + q3)

    joints = (q1, q2, q3, q4)
    if not _within(joints, geom):
        return (None, ANGLE_LIMIT)
    return (joints, OK)


def solve_hand(target, gripper_pitch: float = 0.0,
               geom: ArmGeometry = DEFAULT_GEOMETRY):
    """握手用：两种肘向都试，返回 ``((joints, elbow_up), OK)`` 或 ``(None, reason)``。

    优先肘朝下：握手是"探出去"的姿态，肘朝下时臂身低于肩手连线，不容易扫到人的身体。
    肘朝上留作备用（目标偏近、肘朝下顶到胸口时才用）。
    """
    first = None
    for elbow_up in (False, True):
        joints, reason = inverse(target, gripper_pitch, geom, elbow_up=elbow_up)
        if joints is not None:
            return ((joints, elbow_up), OK)
        if first is None:
            first = reason
    return (None, first or ANGLE_LIMIT)


def check_point(target, gripper_pitch: float = 0.0,
                geom: ArmGeometry = DEFAULT_GEOMETRY) -> str:
    """只判可达性，返回原因字符串（``OK`` 或某类不可达）。"""
    return solve_hand(target, gripper_pitch, geom)[1]


def horizontal_reach_m(height_m: float,
                       geom: ArmGeometry = DEFAULT_GEOMETRY,
                       gripper_pitch: float = 0.0) -> float:
    """在离地 ``height_m`` 处、夹爪保持给定仰角时，水平方向能伸出多远。

    ⚠️ 不是 ``sqrt(reach² − dz²)``：夹爪要先扣掉自己那 4 cm 再横伸。用错会**高估**
    可达范围，现场表现是"规划说够得着，实际差 3~4 cm"。
    """
    dz = height_m - geom.shoulder_height_m
    # 腕相对肩的偏移（扣掉夹爪）
    zw = dz - geom.l_grip * math.sin(gripper_pitch)
    rw_max = geom.l_upper + geom.l_fore
    r2 = rw_max ** 2 - zw ** 2
    if r2 <= 0.0:
        return 0.0
    return math.sqrt(r2) + geom.l_grip * math.cos(gripper_pitch)


def horizontal_band_m(geom: ArmGeometry = DEFAULT_GEOMETRY,
                      gripper_pitch: float = 0.0):
    """夹爪允许水平伸直时，地面高度上真正可达的区间 ``(lo, hi)``。

    要点：两头都要扣夹爪的竖分量，不是 ``shoulder ± reach``。
    """
    zw_max = geom.l_upper + geom.l_fore
    z_off = geom.l_grip * math.sin(gripper_pitch)
    return (geom.shoulder_height_m + z_off - zw_max,
            geom.shoulder_height_m + z_off + zw_max)


def with_shoulder_height(height_m: float,
                         geom: ArmGeometry = DEFAULT_GEOMETRY) -> ArmGeometry:
    """改肩高（上层折叠机构展开/收起），其余不动。"""
    return replace(geom, shoulder_height_m=float(height_m))
