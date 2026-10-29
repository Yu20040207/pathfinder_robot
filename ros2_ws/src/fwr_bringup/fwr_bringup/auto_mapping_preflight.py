"""Physical automatic-mapping preflight; read-only until shell explicitly arms."""
import argparse
from dataclasses import dataclass
import math
import time

from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Int32, Float64MultiArray
from tf2_ros import Buffer, TransformListener, TransformException


@dataclass
class PreflightSnapshot:
    scan_age: float = math.inf
    mapping_scan_age: float = math.inf
    mapping_health_age: float = math.inf
    mapping_healthy: bool = False
    mapping_publisher_count: int = 0
    mapping_publisher_is_filter: bool = False
    imu_age: float = math.inf
    odom_age: float = math.inf
    map_age: float = math.inf
    base_state_age: float = math.inf
    tf_ok: bool = False
    monitor_forward_active: bool = False
    monitor_reverse_active: bool = False
    selector_present: bool = False
    safe_publisher_count: int = 0
    safe_publisher_is_selector: bool = False
    nav_ready: bool = False
    fault: int = -1
    keyboard_running: bool = True
    manual_enabled: bool = True
    motion_enabled: bool = True
    measured_length_m: float = math.inf
    measured_width_m: float = math.inf
    stop_distance_m: float = math.inf


def evaluate_preflight(s):
    errors = []
    for name, age, limit in [('scan', s.scan_age, .25),
                             ('mapping scan', s.mapping_scan_age, .25),
                             ('mapping filter health', s.mapping_health_age, .25),
                             ('IMU', s.imu_age, .25),
                             ('odom', s.odom_age, .25), ('map', s.map_age, 1.),
                             ('base state', s.base_state_age, .25)]:
        if not math.isfinite(age) or age < 0 or age > limit:
            errors.append(f'{name} stale: {age:.2f}s')
    if not s.tf_ok:
        errors.append('map -> base_footprint TF unavailable')
    if not s.mapping_healthy:
        errors.append('mapping scan filter unhealthy')
    if s.mapping_publisher_count != 1 or not s.mapping_publisher_is_filter:
        errors.append('mapping scan has missing or unexpected publisher')
    if not s.monitor_forward_active:
        errors.append('forward Collision Monitor inactive')
    if not s.monitor_reverse_active:
        errors.append('reverse Collision Monitor inactive')
    if not s.selector_present or s.safe_publisher_count != 1 or not s.safe_publisher_is_selector:
        errors.append('directional safety selector missing or safe topic has another publisher')
    if not s.nav_ready:
        errors.append('Nav2 NavigateToPose inactive')
    if s.fault != 0:
        errors.append(f'base fault {s.fault}')
    if s.keyboard_running or s.manual_enabled:
        errors.append('keyboard/manual controller active')
    if s.motion_enabled:
        errors.append('base motion gate already open')
    if (not math.isfinite(s.measured_length_m) or s.measured_length_m <= 0 or
            not math.isfinite(s.measured_width_m) or s.measured_width_m <= 0 or
            s.measured_length_m > .60 or s.measured_width_m > .43):
        errors.append('measured chassis exceeds configured 0.60 x 0.43 m footprint')
    if not math.isfinite(s.stop_distance_m) or s.stop_distance_m > .30 or s.stop_distance_m < 0:
        errors.append('stopping distance exceeds provisional 0.30 m clearance')
    return not errors, errors


class Probe(Node):
    def __init__(self, length_m, width_m, stop_m):
        super().__init__('auto_mapping_preflight')
        self.snapshot = PreflightSnapshot(measured_length_m=length_m,
                                          measured_width_m=width_m,
                                          stop_distance_m=stop_m)
        self._last = {}
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._monitors = {
            'forward': self.create_client(GetState, '/collision_monitor_forward/get_state'),
            'reverse': self.create_client(GetState, '/collision_monitor_reverse/get_state'),
        }
        self._nav_lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self._params = self.create_client(GetParameters, '/base_bridge/get_parameters')
        self.create_subscription(LaserScan, '/scan', lambda m: self._observe('scan', m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_mapping',
                                 lambda m: self._observe('mapping_scan', m), qos_profile_sensor_data)
        self.create_subscription(Bool, '/scan_mapping/healthy', self._mapping_health, 10)
        self.create_subscription(Imu, '/imu/data', lambda m: self._observe('imu', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: self._observe('odom', m), 10)
        self.create_subscription(OccupancyGrid, '/map', lambda m: self._observe('map', m), 10)
        self.create_subscription(Int32, '/base/fault', self._fault, 10)
        self.create_subscription(Float64MultiArray, '/base/motor_debug',
                                 lambda _: self._last.__setitem__('base_state', (0., time.monotonic())), 10)

    def _observe(self, kind, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        self._last[kind] = (stamp, time.monotonic()) if age >= 0 else (0., 0.)

    def _fault(self, msg):
        self.snapshot.fault = msg.data

    def _mapping_health(self, msg):
        self.snapshot.mapping_healthy = bool(msg.data)
        self._last['mapping_health'] = (0., time.monotonic())

    def finish(self):
        s = self.snapshot
        ros_now = self.get_clock().now().nanoseconds / 1e9
        mono_now = time.monotonic()
        for kind in ('scan', 'mapping_scan', 'imu', 'odom', 'map'):
            stamp, received = self._last.get(kind, (0., 0.))
            age = max(ros_now - stamp, mono_now - received) if stamp > 0 else math.inf
            setattr(s, f'{kind}_age', age)
        s.mapping_health_age = (mono_now - self._last['mapping_health'][1]
                                if 'mapping_health' in self._last else math.inf)
        s.base_state_age = mono_now - self._last['base_state'][1] if 'base_state' in self._last else math.inf
        s.keyboard_running = any('keyboard_teleop' in name for name in self.get_node_names())
        s.nav_ready = False
        if self._nav.server_is_ready() and self._nav_lifecycle.service_is_ready():
            future = self._nav_lifecycle.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.)
            s.nav_ready = bool(future.done() and future.result() is not None and
                               future.result().current_state.label == 'active')
        try:
            self._tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            s.tf_ok = True
        except TransformException:
            s.tf_ok = False
        names = self.get_node_names()
        s.selector_present = 'directional_safety_selector' in names
        publishers = self.get_publishers_info_by_topic('/cmd_vel_auto_safe')
        s.safe_publisher_count = len(publishers)
        s.safe_publisher_is_selector = (len(publishers) == 1 and
                                        publishers[0].node_name == 'directional_safety_selector')
        mapping_publishers = self.get_publishers_info_by_topic('/scan_mapping')
        s.mapping_publisher_count = len(mapping_publishers)
        s.mapping_publisher_is_filter = (len(mapping_publishers) == 1 and
                                         mapping_publishers[0].node_name == 'scan_mapping_filter')
        for direction, client in self._monitors.items():
            active = False
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.)
                active = (future.done() and future.result() is not None and
                          future.result().current_state.label == 'active')
            setattr(s, f'monitor_{direction}_active', active)
        if self._params.service_is_ready():
            request = GetParameters.Request()
            request.names = ['motion_enabled', 'manual_enabled']
            future = self._params.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.)
            if future.done() and future.result() is not None and len(future.result().values) == 2:
                s.motion_enabled = future.result().values[0].bool_value
                s.manual_enabled = future.result().values[1].bool_value
        return evaluate_preflight(s)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--length-mm', required=True, type=float)
    parser.add_argument('--width-mm', required=True, type=float)
    parser.add_argument('--stop-mm', required=True, type=float)
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args.length_mm / 1000, args.width_mm / 1000, args.stop_mm / 1000)
    try:
        deadline = time.monotonic() + 6.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        ok, errors = node.finish()
        print('PREFLIGHT_OK' if ok else 'PREFLIGHT_FAILED: ' + '; '.join(errors))
        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
