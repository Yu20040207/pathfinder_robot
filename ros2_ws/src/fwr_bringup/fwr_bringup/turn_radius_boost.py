"""掉头段临时加大膨胀半径，让规划器把弯画得更大。

背景：车在 U 型掉头处会贴着内弯走（现场反馈"太靠门框"）。转弯半径由**路径**
决定，控制器无法把它走宽，所以只能在规划阶段让"贴内弯"变得更贵 ——
也就是临时加大全局代价地图的 inflation_radius。

设计要点：
- **默认不启用**（`enabled: false`），符合"新功能默认关闭、验证后再开"。
- 只处理"掉头"：比较路径起点方向与前方 `lookahead_m` 处的方向，
  夹角超过 `turn_deg` 才动作；普通转弯不受影响。
- 带滞回：连续 `hold_cycles` 个路径消息都判定为掉头才切换，
  掉头结束后同样连续 `hold_cycles` 次才恢复，避免参数抖来抖去。
- 切换失败只记录日志，不影响导航。
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def path_heading_change(poses, lookahead_m: float = 3.0) -> float | None:
    """路径起点方向 与 前方 lookahead_m 处方向 的夹角（弧度，0~pi）。

    数据不足或路径短于 lookahead 时返回 None。
    """
    if poses is None or len(poses) < 2:
        return None
    pts = [(p.pose.position.x, p.pose.position.y) for p in poses]
    # 起点方向：取第一段有长度的线段
    start_heading = None
    for i in range(1, len(pts)):
        dx, dy = pts[i][0] - pts[0][0], pts[i][1] - pts[0][1]
        if math.hypot(dx, dy) > 0.05:
            start_heading = math.atan2(dy, dx)
            break
    if start_heading is None:
        return None
    # 沿路径累计弧长，找到 lookahead_m 处的点，并取**该处路径的切线方向**
    # （不能用"起点到该点的方位角"：U 型掉头时该点回到起点附近，方位角仍是 0°）
    acc = 0.0
    idx = None
    for i in range(1, len(pts)):
        seg = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        acc += seg
        if acc >= lookahead_m:
            idx = i
            break
    if idx is None:
        return None
    # 切线：优先用 (idx, idx+1) 这一段，没有就用 (idx-1, idx)
    a, b = (idx, idx + 1) if idx + 1 < len(pts) else (idx - 1, idx)
    dx, dy = pts[b][0] - pts[a][0], pts[b][1] - pts[a][1]
    if math.hypot(dx, dy) < 1e-6:
        return None
    far_heading = math.atan2(dy, dx)
    d = abs(math.atan2(math.sin(far_heading - start_heading),
                       math.cos(far_heading - start_heading)))
    return d


class TurnRadiusBoost(Node):
    def __init__(self) -> None:
        super().__init__('turn_radius_boost')
        self.declare_parameter('enabled', False)
        self.declare_parameter('plan_topic', '/plan')
        # 实测：3 m 前视下直路 0°、缓弯（半径 4 m）43°、U 型掉头 90°。
        # 阈值取 70°，落在两者之间。
        self.declare_parameter('turn_deg', 70.0)
        self.declare_parameter('lookahead_m', 3.0)
        self.declare_parameter('hold_cycles', 3)
        self.declare_parameter('normal_radius', 0.48)
        self.declare_parameter('boost_radius', 0.65)
        self.declare_parameter('costmap_nodes',
                               ['/global_costmap/global_costmap'])
        self._boosted = False
        self._votes = 0
        self._clients = {}
        self.create_subscription(Path, self.get_parameter('plan_topic').value,
                                 self._on_plan, 10)

    def _set_radius(self, radius: float) -> bool:
        ok = True
        for name in self.get_parameter('costmap_nodes').value:
            cli = self._clients.get(name)
            if cli is None:
                cli = self.create_client(SetParameters, f'{name}/set_parameters')
                self._clients[name] = cli
            if not cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn(f'{name} 参数服务不可用')
                ok = False
                continue
            req = SetParameters.Request()
            p = Parameter()
            p.name = 'inflation_layer.inflation_radius'
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = float(radius)
            req.parameters = [p]
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
            res = fut.result()
            good = bool(res and res.results and res.results[0].successful)
            if not good:
                self.get_logger().warn(f'{name} 设置 inflation 失败')
            ok = ok and good
        return ok

    def _on_plan(self, msg: Path) -> None:
        if not bool(self.get_parameter('enabled').value):
            return
        change = path_heading_change(
            msg.poses, float(self.get_parameter('lookahead_m').value))
        if change is None:
            return
        want = change >= math.radians(float(self.get_parameter('turn_deg').value))
        hold = int(self.get_parameter('hold_cycles').value)
        if want == self._boosted:
            self._votes = 0
            return
        self._votes += 1
        if self._votes < hold:
            return
        self._votes = 0
        radius = (self.get_parameter('boost_radius').value if want
                  else self.get_parameter('normal_radius').value)
        if self._set_radius(radius):
            self._boosted = want
            self.get_logger().info(
                f'{"掉头段：加大膨胀" if want else "普通路段：恢复膨胀"} '
                f'inflation_radius={radius:.2f}（夹角 {math.degrees(change):.0f}°）')


def main() -> None:
    rclpy.init()
    node = TurnRadiusBoost()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
