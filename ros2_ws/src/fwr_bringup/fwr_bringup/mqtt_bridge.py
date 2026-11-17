"""MQTT ↔ ROS 2 桥：手机 App 远程控制的机器人端。

架构（阶段 1）：
    App ──WSS──► 云端 FastAPI ──MQTT──► 本节点 ──/cmd_vel_manual──► base_bridge
                                       ◄──/base/battery, /amcl_pose 等──

MQTT 主题（device_id 由参数给出）：
    robot/{id}/cmd     云端 → 车    {"seq":1,"linear":0.12,"angular":0.0,"ttl_ms":300}
    robot/{id}/state   车 → 云端    {"ts":...,"battery":24.1,"x":1.2,"y":-3.4,"yaw":0.5,...}
    robot/{id}/event   车 → 云端    故障/低电/急停事件
    robot/{id}/online  车 → 云端    retained "online"/"offline"（LWT）

安全：
  * 手动通道（/cmd_vel_manual）：受 base_bridge 的运动门、传感器、安全链保护
  * deadman：超过 cmd_timeout_s 没收到新指令就发零速
  * 速度上限：max_linear_mps / max_angular_rps 双重钳位
  * ttl_ms：过期指令直接丢弃
  * seq：重复序号丢弃
"""
import base64
import gzip
import json
import math
import os
import subprocess
import threading
import time
from collections import deque

import rclpy
import tf2_ros
from rclpy.time import Time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import (Point, Pose, PoseStamped,
                               PoseWithCovarianceStamped,
                               Quaternion)
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, Header, Int32, String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters

try:
    from fwr_interfaces.srv import (StartFixedGoal, StartMultiPointGoal,
                                    StartTwoPointGoal)
except ImportError:                      # 允许无 fwr_interfaces 时降级
    StartFixedGoal = StartMultiPointGoal = StartTwoPointGoal = None

try:
    import paho.mqtt.client as mqtt
except ImportError:                      # pragma: no cover
    mqtt = None


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def parse_command(payload: bytes, max_linear: float, max_angular: float):
    """解析并校验一条控制指令；非法返回 None。"""
    try:
        obj = json.loads(payload.decode('utf-8'))
        linear = float(obj['linear'])
        angular = float(obj['angular'])
    except (ValueError, KeyError, TypeError, UnicodeDecodeError):
        return None
    if not (math.isfinite(linear) and math.isfinite(angular)):
        return None
    seq = obj.get('seq')
    ttl_ms = obj.get('ttl_ms', 300)
    try:
        ttl_ms = max(0, min(2000, int(ttl_ms)))
    except (TypeError, ValueError):
        ttl_ms = 300
    return {
        'seq': seq if isinstance(seq, int) else None,
        'linear': clamp(linear, max_linear),
        'angular': clamp(angular, max_angular),
        'ttl_s': ttl_ms / 1000.0,
    }


TEMP_PATHS = {
    'cpu': '/sys/class/thermal/thermal_zone0/temp',
    'wifi': '/sys/class/thermal/thermal_zone2/temp',
}


def read_temps():
    """读主控温度（CPU / WiFi / NVMe，单位 °C，读不到返回 None）。"""
    out = {}
    try:
        types = {}
        for i in range(8):
            t = f'/sys/class/thermal/thermal_zone{i}'
            try:
                with open(f'{t}/type', encoding='utf-8') as fh:
                    types[i] = fh.read().strip()
            except OSError:
                continue
        want = {'x86_pkg_temp': 'cpu', 'iwlwifi_1': 'wifi', 'acpitz': 'board'}
        for i, name in types.items():
            key = want.get(name)
            if key is None:
                continue
            with open(f'/sys/class/thermal/thermal_zone{i}/temp') as fh:
                out[key] = round(int(fh.read().strip()) / 1000.0, 1)
        for h in range(8):
            base = f'/sys/class/hwmon/hwmon{h}'
            try:
                with open(f'{base}/name', encoding='utf-8') as fh:
                    nm = fh.read().strip()
                if nm == 'nvme':
                    with open(f'{base}/temp1_input') as fh:
                        out['ssd'] = round(int(fh.read().strip()) / 1000.0, 1)
            except OSError:
                continue
    except Exception:                            # noqa: BLE001
        pass
    return out


def build_state_payload(battery, pose, nav_state, fault, armed, ready=False,
                       manual=False, map_name='', speed=0.0, yaw_rate=0.0,
                       leg=0, legs=0):
    """组装上行状态；缺失字段用 None，云端按需处理。"""
    payload = {'ts': time.time()}
    if battery is not None:
        payload['voltage'] = round(battery.voltage, 2)
        payload['current'] = round(battery.current, 2)
        payload['percentage'] = round(battery.percentage, 3)
        payload['charging'] = (battery.power_supply_status ==
                               battery.POWER_SUPPLY_STATUS_CHARGING)
    if pose is not None:
        payload['x'] = round(pose[0], 3)
        payload['y'] = round(pose[1], 3)
        payload['yaw'] = round(pose[2], 3)
    payload['nav'] = nav_state or 'unknown'
    payload['fault'] = int(fault or 0)
    payload['armed'] = bool(armed)
    payload['ready'] = bool(ready)
    payload['manual'] = bool(manual)
    payload['speed'] = round(float(speed), 3)
    temps = read_temps()
    if temps:
        payload['temps'] = temps
    payload['yaw_rate'] = round(float(yaw_rate), 3)
    if legs:
        payload['leg'] = int(leg)
        payload['legs'] = int(legs)
    if map_name:
        payload['map'] = str(map_name)
    return json.dumps(payload, separators=(',', ':')).encode('utf-8')


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 1883)
        self.declare_parameter('device_id', 'robot-001')
        self.declare_parameter('device_secret', '')
        self.declare_parameter('tls', False)
        self.declare_parameter('ca_cert', '')
        self.declare_parameter('max_linear_mps', 0.15)
        self.declare_parameter('max_angular_rps', 0.6)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('cmd_timer_hz', 20.0)
        self.declare_parameter('state_hz', 10.0)
        self.declare_parameter('scan_hz', 5.0)
        self.declare_parameter('scan_points', 180)
        self.declare_parameter('rear_mask_deg', 50.0)
        self.declare_parameter('scan_frame', 'map')
        self.declare_parameter('insecure_tls', False)
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_state_file', '~/.config/fwr/last_navigation_map')

        self._host = str(self.get_parameter('host').value)
        self._port = int(self.get_parameter('port').value)
        self._device = str(self.get_parameter('device_id').value)
        self._secret = str(self.get_parameter('device_secret').value)
        self._use_tls = bool(self.get_parameter('tls').value)
        self._ca = str(self.get_parameter('ca_cert').value)
        self._max_v = float(self.get_parameter('max_linear_mps').value)
        self._max_w = float(self.get_parameter('max_angular_rps').value)
        self._timeout = float(self.get_parameter('cmd_timeout_s').value)

        self._lock = threading.Lock()
        self._cmd = None                 # (linear, angular, deadline_mono)
        self._last_seq = None
        self._active = False
        self._battery = None
        self._pose = None
        self._nav = 'unknown'
        self._fault = 0
        self._armed = False
        self._ready = False
        self._manual = False
        self._connected = False
        self._zero_sent = True

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self._map_published = False
        self._pending = deque()
        self._last_nav_action = ''
        self._busy = False
        self._srv_clients = {}
        if StartFixedGoal is not None:
            self._srv_clients['fixed'] = self.create_client(
                StartFixedGoal, '/explore/start_fixed_goal')
            self._srv_clients['multi'] = self.create_client(
                StartMultiPointGoal, '/explore/start_multi_point_goal')
            self.create_client(StartTwoPointGoal, '/explore/start_two_point_goal')
        self._param_client = self.create_client(
            SetParameters, '/base_bridge/set_parameters')
        self._arm_client = self.create_client(Trigger, '/base/arm_auto')
        self._reseed_client = self.create_client(Trigger, '/localization/reseed')
        self._save_client = self.create_client(Trigger, '/mapping/save_slam')
        self._load_map_client = self.create_client(LoadMap,
                                                   '/map_server/load_map')
        self._saveparam_client = self.create_client(
            SetParameters, '/slam_map_save/set_parameters')
        self._preview_client = ActionClient(self, ComputePathToPose,
                                            '/compute_path_to_pose')
        self._preview_multi = ActionClient(self, ComputePathThroughPoses,
                                           '/compute_path_through_poses')
        self._stop_client = self.create_client(Trigger, '/explore/stop')
        self._resume_client = self.create_client(Trigger, '/explore/resume')
        self._goal_result_pub = None
        self._path_points = None
        self._path_dirty = False
        self.create_timer(0.1, self._drain_actions)
        self.create_timer(1.0, self._publish_path)
        self.create_timer(0.1, self._publish_scan)

        self.create_subscription(BatteryState, '/base/battery',
                                 self._on_battery, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._on_pose, 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._scan_n = 0
        self._scan_last = 0.0
        self._rear_mask_rad = math.radians(float(
            self.get_parameter('rear_mask_deg').value))
        self._scan_points = None
        self._scan_dirty = False
        self._live_map = None
        self._live_map_dirty = False
        self._live_map_at = 0.0
        self._map_name = ''
        self._map_name_at = 0.0
        self._mapping_mode = False
        self._leg = 0
        self._legs = 0
        self._slam_map_seen = False
        self._mapping_grace_until = 0.0
        self._mapping_suppress_until = 0.0
        self._speed = 0.0
        self._yaw_rate = 0.0
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self._local_points = None
        self._local_dirty = False
        self.create_subscription(RosPath, '/local_plan', self._on_local, 10)
        self.create_subscription(RosPath, '/plan', self._on_plan, 10)
        self.create_subscription(OccupancyGrid, '/slam_map', self._on_live_map, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_live_map_fallback, 10)
        self._nav_map_size = None
        self._load_check_timer = None
        self._load_check_before = None
        self._load_check_path = None
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                               QoSProfile, ReliabilityPolicy)
        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_nav_map, map_qos)
        self.create_timer(1.0, self._publish_live_map)
        self.create_timer(5.0, self._sync_mapping_mode)
        self.create_timer(0.5, self._publish_local)
        self.create_subscription(Int32, '/base/fault', self._on_fault, 10)
        self.create_subscription(Bool, '/base/auto_armed', self._on_armed, 10)
        self.create_subscription(Bool, '/base/ready', self._on_ready, 10)
        self.create_subscription(Odometry, '/wheel/odom', self._on_odom, 20)
        self._getparam_client = self.create_client(
            GetParameters, '/base_bridge/get_parameters')
        self.create_timer(2.0, self._poll_manual_flag)
        self.create_subscription(String, '/explore/state', self._on_nav, 10)
        self.create_subscription(String, '/explore/progress', self._on_progress, 10)

        if mqtt is None:
            raise RuntimeError('缺少 paho-mqtt，请 pip install paho-mqtt')
        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f'{self._device}-{int(time.time())}',
            protocol=mqtt.MQTTv311)
        if self._secret:
            self._client.username_pw_set(self._device, self._secret)
        if self._use_tls:
            self._client.tls_set(ca_certs=self._ca or None)
            if bool(self.get_parameter('insecure_tls').value):
                self._client.tls_insecure_set(True)
        self._topic_cmd = f'robot/{self._device}/cmd'
        self._topic_goal = f'robot/{self._device}/goal'
        self._topic_nav = f'robot/{self._device}/nav'
        self._topic_goal_result = f'robot/{self._device}/goal_result'
        self._topic_map = f'robot/{self._device}/map'
        self._topic_path = f'robot/{self._device}/path'
        self._topic_scan = f'robot/{self._device}/scan'
        self._topic_local = f'robot/{self._device}/local_plan'
        self._topic_preview = f'robot/{self._device}/preview_path'
        self._topic_preview_cmd = f'robot/{self._device}/preview'
        self._topic_power = f'robot/{self._device}/power'
        self._topic_live_map = f'robot/{self._device}/live_map'
        self._topic_maps = f'robot/{self._device}/maps'
        self._topic_state = f'robot/{self._device}/state'
        self._topic_event = f'robot/{self._device}/event'
        self._topic_online = f'robot/{self._device}/online'
        self._client.will_set(self._topic_online, 'offline', retain=True)
        self._client.on_connect = self._on_mqtt_connect
        self._client.on_disconnect = self._on_mqtt_disconnect
        self._client.on_message = self._on_mqtt_message

        cmd_period = 1.0 / max(1.0, float(self.get_parameter('cmd_timer_hz').value))
        state_period = 1.0 / max(0.2, float(self.get_parameter('state_hz').value))
        self.create_timer(cmd_period, self._publish_cmd)
        self.create_timer(state_period, self._publish_state)
        self.create_timer(30.0, self._ensure_connected)

        try:
            self._client.connect_async(self._host, self._port, keepalive=30)
            self._client.loop_start()
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().error(f'MQTT 连接启动失败: {exc}')
        self.get_logger().info(
            f'mqtt_bridge 启动：{self._host}:{self._port} device={self._device}')

    # ---------------- MQTT 回调 ----------------
    def _on_mqtt_connect(self, client, _userdata, _flags, reason_code, _props=None):
        self._connected = reason_code == 0
        if self._connected:
            client.publish(self._topic_online, 'online', retain=True)
            client.subscribe(self._topic_cmd, qos=0)
            client.subscribe(self._topic_goal, qos=1)
            client.subscribe(self._topic_nav, qos=1)
            client.subscribe(self._topic_preview_cmd, qos=0)
            client.subscribe(self._topic_power, qos=1)
            if not self._map_published:
                self._publish_map()
            self.get_logger().info('MQTT 已连接')
        else:
            self.get_logger().error(f'MQTT 连接失败: {reason_code}')

    def _on_mqtt_disconnect(self, _client, _userdata, _flags, reason_code, _props=None):
        self._connected = False
        self.get_logger().warn(f'MQTT 断开: {reason_code}')
        with self._lock:
            self._cmd = None

    def _on_mqtt_message(self, _client, _userdata, msg):
        if msg.topic == self._topic_goal:
            self._handle_goal(msg.payload)
            return
        if msg.topic == self._topic_nav:
            self._handle_nav(msg.payload)
            return
        if msg.topic == self._topic_preview_cmd:
            self._handle_preview(msg.payload)
            return
        if msg.topic == self._topic_power:
            self._handle_power(msg.payload)
            return
        if msg.topic != self._topic_cmd:
            return
        parsed = parse_command(msg.payload, self._max_v, self._max_w)
        if parsed is None:
            self.get_logger().warn('收到非法控制指令，已丢弃',
                                   throttle_duration_sec=5.0)
            return
        if parsed['seq'] is not None and parsed['seq'] == self._last_seq:
            return
        self._last_seq = parsed['seq'] if parsed['seq'] is not None else self._last_seq
        with self._lock:
            self._cmd = (parsed['linear'], parsed['angular'],
                         time.monotonic() + parsed['ttl_s'])

    # ---------------- ROS 订阅 ----------------
    def _on_battery(self, msg):
        self._battery = msg

    def _on_pose(self, msg):
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y,
                      yaw_of(p.orientation))

    def _on_fault(self, msg):
        if msg.data and msg.data != self._fault:
            self._publish_event('fault', {'code': int(msg.data)})
        self._fault = int(msg.data)

    def _on_armed(self, msg):
        self._armed = bool(msg.data)

    def _on_ready(self, msg):
        self._ready = bool(msg.data)

    def _on_odom(self, msg):
        self._speed = float(msg.twist.twist.linear.x)
        self._yaw_rate = float(msg.twist.twist.angular.z)

    def _poll_manual_flag(self):
        """定期读 /base_bridge 的 manual_enabled，用于手机端“已授权/未授权”。"""
        client = self._getparam_client
        if not client.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ['manual_enabled']
        client.call_async(req).add_done_callback(self._on_manual_flag)

    def _on_manual_flag(self, future):
        try:
            res = future.result()
            if res and res.values:
                self._manual = bool(res.values[0].bool_value)
        except Exception:                        # noqa: BLE001
            pass

    def _on_progress(self, msg):
        try:
            data = json.loads(msg.data)
            self._leg = int(data.get('leg', 0))
            self._legs = int(data.get('total', 0))
        except (ValueError, TypeError, AttributeError):
            return

    def _on_nav(self, msg):
        self._nav = msg.data

    # ---------------- 定时器 ----------------
    def _publish_cmd(self):
        now = time.monotonic()
        with self._lock:
            cmd = self._cmd
            if cmd is not None and now > cmd[2]:
                self._cmd = None
                cmd = None
        twist = Twist()
        if cmd is not None:
            twist.linear.x = cmd[0]
            twist.angular.z = cmd[1]
            self._zero_sent = False
        elif not self._zero_sent:
            self._zero_sent = True
            self.get_logger().info('指令超时，停车')
        else:
            return
        self._cmd_pub.publish(twist)

    # ---------------- 地图 / 导航目标 ----------------
    def _map_path(self) -> str:
        path = str(self.get_parameter('map_yaml').value or '')
        if path:
            return os.path.expanduser(path)
        state = os.path.expanduser(
            str(self.get_parameter('map_state_file').value))
        try:
            with open(state, 'r', encoding='utf-8') as fh:
                value = fh.read().strip()
            self.get_logger().info(f'地图状态文件 {state} -> {value!r}',
                                   throttle_duration_sec=10.0)
            return value
        except OSError as exc:
            self.get_logger().warn(f'读取地图状态文件失败 {state}: {exc}')
            return ''

    def _publish_map(self) -> None:
        """把最近一次导航用的 PGM 地图压缩后发到云端（retained）。"""
        yaml_path = self._map_path()
        if not yaml_path or not os.path.isfile(yaml_path):
            self.get_logger().warn('没有可用的地图文件，跳过地图上传')
            self._map_published = True
            return
        try:
            with open(yaml_path, 'r', encoding='utf-8') as fh:
                meta = {}
                for line in fh:
                    if ':' in line:
                        key, value = line.split(':', 1)
                        meta[key.strip()] = value.strip()
            image = meta.get('image', '')
            if not os.path.isabs(image):
                image = os.path.join(os.path.dirname(yaml_path), image)
            with open(image, 'rb') as fh:
                raw = fh.read()
            # 解析 P5 头：跳过注释行（map_saver 会写 # CREATOR ...），拿到宽高
            fields = []
            idx = 0
            while len(fields) < 3:
                end = raw.find(b'\n', idx)
                if end < 0:
                    break
                line = raw[idx:end].strip()
                idx = end + 1
                if not line or line.startswith(b'#'):
                    continue
                fields.extend(line.split())
            if len(fields) < 3:
                raise ValueError('PGM 头解析失败')
            width, height = int(fields[1]), int(fields[2])
            payload = {
                'resolution': float(meta.get('resolution', 0.05)),
                'origin': [float(v) for v in
                           meta.get('origin', '[0,0,0]').strip('[]').split(',')],
                'negate': int(float(meta.get('negate', 0))),
                'width': width,
                'height': height,
                'pgm': base64.b64encode(gzip.compress(raw, 6)).decode('ascii'),
                'yaml': os.path.basename(yaml_path),
            }
            self._client.publish(self._topic_map,
                                 json.dumps(payload, separators=(',', ':')),
                                 qos=1, retain=True)
            self._map_published = True
            self.get_logger().info(
                f'地图已上传：{os.path.basename(image)} {width}x{height}')
        except (OSError, ValueError, KeyError) as exc:
            self.get_logger().error(f'上传地图失败: {exc}')

    def _handle_goal(self, payload: bytes) -> None:
        try:
            obj = json.loads(payload.decode('utf-8'))
            points = [[float(p[0]), float(p[1])] for p in obj.get('points', [])]
        except (ValueError, TypeError, IndexError, UnicodeDecodeError):
            self.get_logger().warn('非法的导航目标，已忽略')
            return
        if not 1 <= len(points) <= 20:
            self.get_logger().warn(f'导航点数量 {len(points)} 不合法')
            return
        self.get_logger().info(f'收到导航目标：{len(points)} 个点 {points[:3]}')
        self._pending.append(('arm', points))

    def _handle_nav(self, payload: bytes) -> None:
        try:
            obj = json.loads(payload.decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            return
        action = str(obj.get('action') or '')
        if not action:
            return
        self._pending.append(('nav', obj))

    def _handle_power(self, payload: bytes) -> None:
        """远程关机/重启主控（需要 manbaout 免密 sudo）。"""
        try:
            action = json.loads(payload.decode('utf-8')).get('action', '')
        except (ValueError, UnicodeDecodeError):
            return
        if action not in ('shutdown', 'reboot'):
            self.get_logger().warn(f'未知电源指令: {action}')
            return
        self.get_logger().warn(f'收到电源指令: {action}，先停车再执行')
        try:
            self._cmd_pub.publish(Twist())
        except Exception:                        # noqa: BLE001
            pass
        time.sleep(0.5)
        cmd = ['sudo', '-n', 'shutdown', '-h', 'now'] if action == 'shutdown' \
            else ['sudo', '-n', 'reboot']
        try:
            subprocess.run(cmd, timeout=10, check=False)
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().error(f'执行电源指令失败: {exc}')

    def _handle_preview(self, payload: bytes) -> None:
        """计算到指定点的路径（不执行），用于点击航点后预览路线。"""
        try:
            obj = json.loads(payload.decode('utf-8'))
            pts = [[float(p[0]), float(p[1])] for p in obj.get('points', [])]
        except (ValueError, TypeError, IndexError, UnicodeDecodeError):
            return
        if not pts:
            return
        self._pending.append(('preview', pts))

    def _pose_stamped(self, x: float, y: float) -> PoseStamped:
        return PoseStamped(
            header=Header(stamp=self.get_clock().now().to_msg(),
                          frame_id='map'),
            pose=Pose(position=Point(x=x, y=y, z=0.0),
                      orientation=Quaternion(w=1.0)))

    def _drain_actions(self) -> None:
        """串行处理待执行动作（0.1 秒轮询）。"""
        if self._busy or not self._pending:
            return
        kind, payload = self._pending.popleft()
        if kind == 'preview':
            self._busy = True
            self._start_preview(payload)
            return
        if kind == 'arm':
            self._busy = True
            self._start_arm(payload)
            return
        if kind == 'goal':
            points = payload
            if len(points) == 1:
                client = self._srv_clients.get('fixed')
                if client is None:
                    self._busy = False
                    return
                if not client.service_is_ready():
                    self._busy = False
                    self._pending.appendleft((kind, payload))
                    return
                req = StartFixedGoal.Request(x=points[0][0], y=points[0][1],
                                             yaw=0.0)
                self.get_logger().info(f'调用 /explore/start_fixed_goal ({points[0]})')
                future = client.call_async(req)
                future.add_done_callback(self._on_goal_done)
            else:
                client = self._srv_clients.get('multi')
                if client is None:
                    self._busy = False
                    return
                if not client.service_is_ready():
                    self._busy = False
                    self._pending.appendleft((kind, payload))
                    return
                self.get_logger().info(
                    f'调用 /explore/start_multi_point_goal ({len(points)} 点)')
                req = StartMultiPointGoal.Request(
                    xs=[p[0] for p in points], ys=[p[1] for p in points])
                future = client.call_async(req)
                future.add_done_callback(self._on_goal_done)
            self._busy = True
        elif kind == 'nav':
            obj = payload if isinstance(payload, dict) else {'action': payload}
            action = str(obj.get('action') or '')
            self._last_nav_action = action
            if action == 'manual':
                self._set_gate(True, True, True)
                return
            if action == 'mapping_save':
                name = str(obj.get('name') or '').strip()
                client = self._save_client
                if not client.service_is_ready():
                    self.get_logger().warn('保存地图服务不可用，忽略本次请求')
                    return
                if name:
                    self._set_save_prefix(name, self._do_save)
                else:
                    self._do_save()
                return
            if action == 'list_maps':
                self._publish_map_list()
                return
            if action == 'set_map':
                self._apply_map_choice(str(obj.get('map') or ''))
                return
            if action in ('mapping_start', 'mapping_stop', 'mapping_restart'):
                self._run_service_action(action)
                return
            if action == 'relocalize':
                client = self._reseed_client
                if not client.service_is_ready():
                    self._pending.appendleft((kind, obj))
                    return
                client.call_async(Trigger.Request()).add_done_callback(
                    self._on_nav_done)
                self._busy = True
                return
            client = (self._stop_client if action == 'stop'
                      else self._resume_client if action == 'resume' else None)
            if client is None or not client.service_is_ready():
                self._pending.appendleft((kind, obj))
                return
            future = client.call_async(Trigger.Request())
            future.add_done_callback(self._on_nav_done)
            self._busy = True

    def _start_arm(self, points) -> None:
        """导航前自动开门：motion_enabled=true、关手动、arm_auto。"""
        self.get_logger().info('开始导航授权流程（开门 + arm_auto）')
        if not (self._param_client.service_is_ready()
                and self._arm_client.service_is_ready()):
            self._busy = False
            self._pending.appendleft(('arm', points))
            self.get_logger().warn('等待 /base_bridge 参数服务或 arm_auto 服务…',
                                   throttle_duration_sec=5.0)
            return
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='motion_enabled',
                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                           bool_value=True)),
            Parameter(name='manual_enabled',
                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                           bool_value=False)),
            Parameter(name='manual_allow_mixed',
                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                           bool_value=False)),
        ]
        self._param_client.call_async(req)
        self.get_logger().info('运动门已开（motion=True manual=False mixed=False）')
        self._arm_client.call_async(Trigger.Request()).add_done_callback(
            lambda f: self._on_arm_done(f, points))

    def _on_arm_done(self, future, points) -> None:
        self._busy = False
        ok = False
        try:
            res = future.result()
            ok = bool(getattr(res, 'success', False))
            self.get_logger().info(
                f'arm_auto 结果: {ok} {getattr(res, "message", "")}')
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().warn(f'arm_auto 失败: {exc}')
        if not ok:
            self._publish_result('goal', False, 'arm_auto 失败，未下发目标')
            return
        self._pending.appendleft(('goal', points))

    def _set_gate(self, motion: bool, manual: bool, mixed: bool) -> None:
        if not self._param_client.wait_for_service(timeout_sec=0.3):
            self.get_logger().warn(
                '等待 /base_bridge 参数服务，授权稍后重试',
                throttle_duration_sec=3.0)
            self._pending.appendleft(('nav', {'action': 'manual'}))
            return
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='motion_enabled',
                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                           bool_value=motion)),
            Parameter(name='manual_enabled',
                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                           bool_value=manual)),
            Parameter(name='manual_allow_mixed',
                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                           bool_value=mixed)),
        ]
        self._param_client.call_async(req)
        self.get_logger().info(
            f'运动门设置 motion={motion} manual={manual} mixed={mixed}')

    def _start_preview(self, targets) -> None:
        """1 个点用 ComputePathToPose；多个点用 ComputePathThroughPoses。"""
        if not targets:
            self._busy = False
            return
        if len(targets) == 1:
            client = self._preview_client
            if not client.wait_for_server(timeout_sec=0.2):
                self._busy = False
                self.get_logger().warn('规划器不可用，丢弃本次路径预览',
                                       throttle_duration_sec=5.0)
                return
            goal = ComputePathToPose.Goal()
            goal.goal = self._pose_stamped(targets[0][0], targets[0][1])
            goal.use_start = False
        else:
            client = self._preview_multi
            if not client.wait_for_server(timeout_sec=0.2):
                self._busy = False
                self.get_logger().warn('规划器不可用，丢弃本次路径预览',
                                       throttle_duration_sec=5.0)
                return
            goal = ComputePathThroughPoses.Goal()
            goal.goals = [self._pose_stamped(x, y) for x, y in targets]
            goal.use_start = False
        client.send_goal_async(goal).add_done_callback(self._on_preview_goal)

    def _on_preview_goal(self, future) -> None:
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                self._busy = False
                return
            handle.get_result_async().add_done_callback(
                self._on_preview_result)
        except Exception as exc:                 # noqa: BLE001
            self._busy = False
            self.get_logger().warn(f'路径预览失败: {exc}')

    def _on_preview_result(self, future) -> None:
        self._busy = False
        try:
            result = future.result().result
            pts = [[round(p.pose.position.x, 2), round(p.pose.position.y, 2)]
                   for p in result.path.poses[:500]]
        except Exception:                        # noqa: BLE001
            return
        if not pts or not self._connected:
            return
        body = json.dumps({'points': pts}, separators=(',', ':'))
        self._client.publish(self._topic_preview, body, qos=1)
        self.get_logger().info(f'路径预览已生成：{len(pts)} 个点')

    def _run_service_action(self, action: str) -> None:
        """建图/导航栈切换：后台线程执行 systemctl。"""
        import threading
        cmds = {
            'mapping_start': ['bash', '-lc',
                              'sudo -n systemctl stop fwr-nav.service; '
                              'sleep 2; sudo -n systemctl start fwr-map.service'],
            'mapping_stop': ['bash', '-lc',
                             'sudo -n systemctl stop fwr-map.service; '
                             'sleep 2; sudo -n systemctl start fwr-nav.service'],
            'mapping_restart': ['sudo', '-n', 'systemctl', 'restart',
                                'fwr-map.service'],
            'nav_restart': ['bash', '-lc',
                            'sudo -n systemctl stop fwr-map.service; '
                            'sleep 2; sudo -n systemctl restart fwr-nav.service'],
        }
        cmd = cmds.get(action)
        if cmd is None:
            return
        if action in ('mapping_start', 'mapping_restart'):
            self._mapping_mode = True
            self._mapping_grace_until = time.monotonic() + 25.0
        elif action in ('mapping_stop', 'nav_restart'):
            self._mapping_mode = False
            self._mapping_suppress_until = time.monotonic() + 20.0

        def worker():
            self.get_logger().warn(f'执行 {action}: {" ".join(cmd)}')
            try:
                subprocess.run(cmd, timeout=120, check=False)
                self.get_logger().warn(f'{action} 完成')
                if action in ('nav_restart', 'mapping_stop'):
                    self._map_name_at = 0.0
                    self._publish_map()
                self._publish_result('nav', True, f'{action} 完成',
                                     action=action)
            except Exception as exc:             # noqa: BLE001
                self.get_logger().error(f'{action} 失败: {exc}')
                self._publish_result('nav', False, f'{action} 失败: {exc}',
                                     action=action)

        threading.Thread(target=worker, daemon=True).start()

    def _do_save(self) -> None:
        self._busy = True
        self._save_client.call_async(Trigger.Request()).add_done_callback(
            self._on_nav_done)

    def _set_save_prefix(self, name: str, on_done) -> None:
        """把地图名写给 slam_map_save（文件传递，避免参数服务发现不稳）。"""
        try:
            from pathlib import Path
            Path('/tmp/fwr_map_save_name.txt').write_text(name, encoding='utf-8')
            self.get_logger().info(f'地图名已下发: {name}')
        except OSError as exc:
            self.get_logger().warn(f'写地图名失败: {exc}')
        on_done()

    def _map_files(self):
        from pathlib import Path
        dirs = [Path('/home/manbaout/Company_Project/Four-wheel-robot/'
                     'ros2_ws/src/fwr_bringup/maps'),
                Path.home() / 'robot_maps']
        out = []
        for d in dirs:
            if not d.is_dir():
                continue
            for f in sorted(d.glob('*.yaml')):
                out.append((f.name.replace('.yaml', ''), str(f)))
        return out

    def _publish_map_list(self) -> None:
        maps = [{'name': n, 'path': pth} for n, pth in self._map_files()]
        if not self._connected:
            return
        self._client.publish(self._topic_maps,
                             json.dumps({'maps': maps}, ensure_ascii=False),
                             qos=1)
        self.get_logger().info(f'地图列表已上报：{len(maps)} 个')

    def _apply_map_choice(self, path: str) -> None:
        from pathlib import Path
        if not path:
            return
        p = Path(path)
        if not p.is_file():
            self.get_logger().warn(f'地图不存在: {path}')
            return
        try:
            state = Path.home() / '.config/fwr/last_navigation_map'
            state.parent.mkdir(parents=True, exist_ok=True)
            state.write_text(str(p), encoding='utf-8')
        except OSError as exc:
            self.get_logger().error(f'写入地图状态失败: {exc}')
            self._publish_result('nav', False, f'写入地图状态失败: {exc}',
                                 action='set_map')
            return
        # 运行时 load_map 在此 Nav2 版本不可靠（返回 false 且不重发地图），
        # 直接重启导航栈（页面用进度条等待就绪）
        self.get_logger().info(f'导航地图已切换为 {p.name}，重启导航栈')
        self._publish_result('nav', True, p.stem, action='set_map')
        self._run_service_action('nav_restart')

    def _on_load_map(self, future, path) -> None:
        if self._load_check_timer is not None:
            try:
                self.destroy_timer(self._load_check_timer)
            except Exception:                    # noqa: BLE001
                pass
        self._load_check_before = self._nav_map_size
        self._load_check_path = path
        try:
            future.result()
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().warn(f'load_map 调用失败: {exc}')
        self._load_check_timer = self.create_timer(4.0, self._check_load_map)

    def _check_load_map(self) -> None:
        if self._load_check_timer is not None:
            self.destroy_timer(self._load_check_timer)
            self._load_check_timer = None
        before = self._load_check_before
        path = self._load_check_path
        ok = (self._nav_map_size is not None and
              self._nav_map_size != before)
        if ok:
            self.get_logger().info(f'地图已切换（运行时）: {path.name}')
            self._publish_result('nav', True, path.stem, action='set_map')
        else:
            self.get_logger().warn('load_map 未生效，回退重启导航栈')
            self._publish_result('nav', False, 'load_map 未生效', action='set_map')
            self._run_service_action('nav_restart')

    def _on_nav_map(self, msg) -> None:
        self._nav_map_size = (int(msg.info.width), int(msg.info.height))

    def _on_live_map_fallback(self, msg) -> None:
        """没有 /slam_map 时（旧启动方式）才使用 /map，避免混入其它节点的地图。"""
        if self._slam_map_seen:
            return
        self._on_live_map(msg, from_slam=False)

    def _on_live_map(self, msg, from_slam: bool = True) -> None:
        """SLAM 地图：压缩后低频上传，供手机建图模式显示。"""
        if not self._mapping_mode:
            return
        if time.monotonic() < self._mapping_grace_until:
            return          # 切换宽限期：忽略旧导航栈残留的 /map
        if time.monotonic() < self._mapping_suppress_until:
            return          # 刚停止建图：抑制残留的实时地图
        now = time.monotonic()
        if now - self._live_map_at < 1.0:
            return
        try:
            w, h = int(msg.info.width), int(msg.info.height)
            # OccupancyGrid: -1=未知, 0=空闲, 100=占用；行序从下往上
            gray = bytes(v & 0xFF for v in msg.data)
            rows = [gray[i * w:(i + 1) * w] for i in range(h)]
            top_down = b''.join(reversed(rows))
            table = bytearray(256)
            table[255] = 205          # 未知 → 灰
            table[0] = 254            # 空闲 → 白
            for i in range(1, 101):
                table[i] = 0          # 占用 → 黑
            data = top_down.translate(bytes(table))
            payload = {
                'resolution': float(msg.info.resolution),
                'origin': [float(msg.info.origin.position.x),
                           float(msg.info.origin.position.y), 0.0],
                'width': int(msg.info.width),
                'height': int(msg.info.height),
                'pgm': base64.b64encode(gzip.compress(data, 6)).decode('ascii'),
            }
        except Exception:                        # noqa: BLE001
            return
        if from_slam:
            self._slam_map_seen = True
        self._live_map = payload
        self._live_map_dirty = True
        self._live_map_at = now

    def _sync_mapping_mode(self) -> None:
        try:
            out = subprocess.run(['systemctl', 'is-active', 'fwr-map'],
                                 capture_output=True, text=True, timeout=3)
            self._mapping_mode = out.stdout.strip() == 'active'
        except Exception:                        # noqa: BLE001
            pass

    def _current_map_name(self) -> str:
        now = time.monotonic()
        if now - self._map_name_at < 5.0:
            return self._map_name
        self._map_name_at = now
        try:
            from pathlib import Path
            state = Path.home() / '.config/fwr/last_navigation_map'
            if state.is_file():
                self._map_name = Path(state.read_text(encoding='utf-8').strip()).stem
        except OSError:
            pass
        return self._map_name

    def _publish_live_map(self) -> None:
        if not self._connected or not self._live_map_dirty or self._live_map is None:
            return
        self._live_map_dirty = False
        self._client.publish(self._topic_live_map,
                             json.dumps(self._live_map, separators=(',', ':')),
                             qos=0)

    def _on_local(self, msg) -> None:
        pts = []
        for pose in msg.poses[:80]:
            pts.append([round(pose.pose.position.x, 2),
                        round(pose.pose.position.y, 2)])
        self._local_points = pts
        self._local_dirty = True

    def _publish_local(self) -> None:
        if not self._connected or not self._local_dirty or self._local_points is None:
            return
        self._local_dirty = False
        body = json.dumps({'points': self._local_points},
                          separators=(',', ':'))
        self._client.publish(self._topic_local, body, qos=0)

    def _on_scan(self, msg) -> None:
        """把激光点转到地图坐标、抽稀后发布（默认 5 Hz / 180 点）。"""
        now = time.monotonic()
        min_period = 1.0 / max(0.5, float(
            self.get_parameter('scan_hz').value))
        if now - self._scan_last < min_period:
            return
        self._scan_last = now
        try:
            tf = self._tf_buffer.lookup_transform(
                str(self.get_parameter('scan_frame').value),
                msg.header.frame_id, Time())
        except Exception:                        # noqa: BLE001
            return
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        cos, sin = math.cos(yaw), math.sin(yaw)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        target = max(10, int(self.get_parameter('scan_points').value))
        step = max(1, len(msg.ranges) // target)
        pts = []
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if not math.isfinite(r) or r < 0.12 or r > 8.0:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            # 屏蔽车尾 ±50°（车体自击，与 RViz 的 /scan_mapping 一致）
            wrapped = math.atan2(math.sin(ang), math.cos(ang))
            if abs(wrapped) <= self._rear_mask_rad:
                continue
            ang = wrapped
            lx, ly = r * math.cos(ang), r * math.sin(ang)
            wx = tx + lx * cos - ly * sin
            wy = ty + lx * sin + ly * cos
            pts.append([int(round(wx * 100)), int(round(wy * 100))])
        self._scan_points = pts
        self._scan_dirty = True

    def _publish_scan(self) -> None:
        if not self._connected or not self._scan_dirty or self._scan_points is None:
            return
        self._scan_dirty = False
        body = json.dumps({'points': self._scan_points},
                          separators=(',', ':'))
        self._client.publish(self._topic_scan, body, qos=0)

    def _on_plan(self, msg) -> None:
        pts = []
        for pose in msg.poses[:500]:
            pts.append([round(pose.pose.position.x, 3),
                        round(pose.pose.position.y, 3)])
        self._path_points = pts
        self._path_dirty = True

    def _publish_path(self) -> None:
        if not self._connected or not self._path_dirty:
            return
        self._path_dirty = False
        body = json.dumps({'points': self._path_points or []},
                          separators=(',', ':'))
        self._client.publish(self._topic_path, body, qos=0)

    def _on_goal_done(self, future) -> None:
        self._busy = False
        accepted, reason = False, ''
        try:
            res = future.result()
            accepted, reason = bool(res.accepted), res.reason
        except Exception as exc:                 # noqa: BLE001
            reason = f'服务调用失败: {exc}'
        self.get_logger().info(f'目标结果 accepted={accepted} reason={reason}')
        self._publish_result('goal', accepted, reason)

    def _on_nav_done(self, future) -> None:
        self._busy = False
        ok, reason = False, ''
        try:
            res = future.result()
            ok, reason = bool(res.success), res.message
        except Exception as exc:                 # noqa: BLE001
            reason = f'服务调用失败: {exc}'
        if ok and self._last_nav_action == 'mapping_save':
            saved = self._promote_saved_map()
            if saved:
                reason = f'{reason}（已设为下次导航地图: {saved.name}）'
        self._publish_result('nav', ok, reason)

    def _promote_saved_map(self):
        """把刚保存的最新地图设为下次导航使用的地图。"""
        from pathlib import Path
        dirs = [Path('/home/manbaout/Company_Project/Four-wheel-robot/'
                     'ros2_ws/src/fwr_bringup/maps'),
                Path.home() / 'robot_maps']
        best = None
        for d in dirs:
            if not d.is_dir():
                continue
            for f in d.glob('*.yaml'):
                if best is None or f.stat().st_mtime > best.stat().st_mtime:
                    best = f
        if best is None:
            return None
        try:
            state = Path.home() / '.config/fwr/last_navigation_map'
            state.parent.mkdir(parents=True, exist_ok=True)
            state.write_text(str(best), encoding='utf-8')
            self.get_logger().info(f'下次导航地图已切换为 {best}')
        except OSError as exc:
            self.get_logger().warn(f'写入导航地图状态失败: {exc}')
            return None
        return best

    def _publish_result(self, kind: str, accepted: bool, reason: str,
                        action: str = '') -> None:
        if not self._connected:
            return
        act = action or (self._last_nav_action if kind == 'nav' else '')
        body = json.dumps({'kind': kind, 'accepted': bool(accepted),
                           'reason': reason, 'action': act,
                           'ts': time.time()},
                          ensure_ascii=False)
        self._client.publish(self._topic_goal_result, body, qos=1)

    def _pose_from_tf(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', Time())
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        except Exception:                        # noqa: BLE001
            return None

    def _publish_state(self):
        if not self._connected:
            return
        pose = self._pose
        if self._mapping_mode:
            tf_pose = self._pose_from_tf()
            if tf_pose is not None:
                pose = tf_pose
        payload = build_state_payload(self._battery, pose,
                                      self._nav, self._fault, self._armed,
                                      self._ready, self._manual,
                                      self._current_map_name(),
                                      self._speed, self._yaw_rate,
                                      self._leg, self._legs)
        self._client.publish(self._topic_state, payload, qos=0)

    def _publish_event(self, kind: str, data: dict):
        if not self._connected:
            return
        body = {'ts': time.time(), 'kind': kind}
        body.update(data)
        self._client.publish(self._topic_event,
                             json.dumps(body, separators=(',', ':')),
                             qos=1)

    def _ensure_connected(self):
        if self._connected:
            return
        try:
            self._client.reconnect()
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().warn(f'MQTT 重连失败: {exc}',
                                   throttle_duration_sec=30.0)

    def destroy_node(self):
        try:
            self._client.publish(self._topic_online, 'offline', retain=True)
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:                        # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
