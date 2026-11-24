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
from pathlib import Path
import socket
import subprocess
import threading
import time
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
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
    ttl_ms = obj.get('ttl_ms', 800)
    try:
        ttl_ms = max(0, min(2000, int(ttl_ms)))
    except (TypeError, ValueError):
        ttl_ms = 800
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
                       leg=0, legs=0, front_at=0.0, rear_at=0.0, mode=''):
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
    now_m = time.monotonic()
    # 2026-11-19：窗口从 2 s 放宽到 3 s。2 s 时雷达偶尔掉一两帧
    # 就会让手机端"雷达就绪"标记来回跳（现场看到"已就绪↔等待"刷屏）。
    payload['lidar_front'] = (now_m - front_at) < 3.0
    payload['lidar_rear'] = (now_m - rear_at) < 3.0
    if legs:
        payload['leg'] = int(leg)
        payload['legs'] = int(legs)
    if map_name:
        payload['map'] = str(map_name)
    # 2026-11-19：车端实际在跑哪个栈。页面只凭本地 lastMode 判断时，
    # 切换指令丢失会出现"页面显示建图探索、车端仍在导航栈"（11:44 黑屏不出图）。
    if mode:
        payload['mode'] = str(mode)
    return json.dumps(payload, separators=(',', ':')).encode('utf-8')


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def _system_shutting_down() -> bool:
    """系统是否正在关机/重启。

    systemd 排定关机时会写 `/run/systemd/shutdown/scheduled`；这个文件比
    监听 SIGTERM 可靠（我们可能在信号到达前就被重复拉起）。
    """
    try:
        return os.path.exists('/run/systemd/shutdown/scheduled')
    except OSError:
        return False


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
        self.declare_parameter('scan_points', 360)
        self.declare_parameter('rear_mask_deg', 50.0)
        self.declare_parameter('scan_merged_topic', '/scan_merged')
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
        self._busy_since = 0.0
        self._slam_map_w = -1          # 最近一帧 SLAM 地图宽度
        self._empty_map_since = 0.0    # 从什么时候开始一直 0×0
        self._last_map_recover = 0.0   # 上次自动重启建图栈的时间
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
        self._desired_gate = None      # 待生效的 (motion, manual, mixed)
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
        self.create_timer(1.0, self._apply_pending_gate)
        self.create_timer(1.0, self._publish_path)
        self.create_timer(0.1, self._publish_scan)

        self.create_subscription(BatteryState, '/base/battery',
                                 self._on_battery, 10)
        # 2026-11-20：安全链心跳。手机点"开始导航"时如果 collision_monitor 还没
        # active（导航栈重启后要等 ~50 s 才激活），Nav2 会 15 s 后报
        # "Failed to make progress" 放弃目标 —— 现场表现就是"换完地图点导航，
        # 车停一下又不动"。这里记录过滤后速度的最后时刻，状态里给手机一个
        # chain 标志，让前端等链通再发目标。
        self.create_subscription(Twist, '/cmd_vel_auto_forward_filtered',
                                 self._on_filtered_vel, 10)
        self._last_filtered = 0.0
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
        # 2026-11-20 用户反馈：切图时"地图已切换"的提示比地图实际生效早。
        # 原因：_current_map_name() 读的是状态文件，而 _apply_map_choice 一开始
        # 就把它写了，于是手机立刻以为切好了（其实导航栈还在重启）。
        # 现在分开两个概念：
        #   _active_map_path = map_server 真正加载的那张（页面只认它）
        #   _pending_map_path = 刚选好、等重启完成才生效的那张
        self._active_map_path = ''
        self._pending_map_path = ''
        self._mapping_mode = False
        # 2026-11-24：模式改成"开机手动 + 按需启栈"后，手机要能区分
        # manual / nav / map / follow（原来只有 map/nav 两态）。
        self._current_mode = 'manual'
        self._mode_lock = threading.Lock()
        self._mode_running = False
        self._mode_running_action = ''
        self._mode_pending = ''
        self._leg = 0
        self._legs = 0
        # 当前真实目标坐标（/explore/progress 带过来），手机按它反查航点编号
        self._leg_x = None
        self._leg_y = None
        self._leg_at = 0.0
        self._slam_map_seen = False
        self._mapping_grace_until = 0.0
        self._mapping_suppress_until = 0.0
        self._speed = 0.0
        self._yaw_rate = 0.0
        self._merged_at = 0.0
        merged_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_merged_topic').value),
            self._on_scan_merged, merged_qos)
        raw_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self._front_at = 0.0
        self._rear_at = 0.0
        self.create_subscription(LaserScan, '/scan', self._on_scan_front, 10)
        self.create_subscription(LaserScan, '/scan_rear', self._on_scan_rear,
                                 raw_qos)
        self._local_points = None
        self._local_dirty = False
        self.create_subscription(RosPath, '/local_plan', self._on_local, 10)
        self.create_subscription(RosPath, '/plan', self._on_plan, 10)
        # 2026-11-19：建图地图用 TRANSIENT_LOCAL 订阅。
        # `/map` 由 maperaser 以 TRANSIENT_LOCAL 发布（低频、只在变化时发），
        # 桥接原来用默认 volatile：如果它比 maperaser 晚订阅，
        # 就会一直收不到那唯一一帧 → 手机建图探索一直黑屏（11:44 现场）。
        map_qos = QoSProfile(depth=1,
                             history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/slam_map', self._on_live_map,
                                 map_qos)
        self.create_subscription(OccupancyGrid, '/map',
                                 self._on_live_map_fallback, map_qos)
        self._nav_map_size = None
        self._published_map_size = None      # 最近一次发到云端的图尺寸
        self._pending_map_size = None        # 切图中"应该加载"的图尺寸
        self._nav_map_msgs = 0          # 收到多少帧 /map（换图后要看到刷新）
        self._load_check_timer = None
        self._load_check_before = None
        self._load_check_path = None
        map_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_nav_map, map_qos)
        self._topic_lmap_plain = f'robot/{self._device}/lmap_plain'
        self._topic_map_edit = f'robot/{self._device}/map_edit'
        self._live_map_plain = False
        self._last_occ = None
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
        # 启动时确定"当前已加载的地图"（导航栈正是按状态文件起的）
        self._init_active_map()
        # 2026-11-21：以前 client_id 带启动时间戳 → 每次重启都是一个全新会话，
        # 被杀掉的老进程要等 keepalive(30 s) 才由 broker 结算 LWT，
        # 那条 retained "offline" 会把新进程刚发的 "online" 冲掉
        # （现场表现：机器明明在线，手机却显示设备离线）。
        # 改成固定 client_id：新连接一进来就接管旧会话，旧 LWT 当场结算。
        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f'fwr-{self._device}',
            clean_session=True,
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
        # 2026-11-21：用户点关机后，systemd 的 Restart=always 有可能在关机
        # 过程中把本进程再拉起一次 → 重新发 retained "online" → 手机端
        # 先显示离线、又显示"设备已重新上线"、过一会才真正离线。
        # 这个标志 + /run/systemd/shutdown/scheduled 检查一起堵住它。
        self._shutting_down = False
        self._online_timer = None
        self._client.will_set(self._topic_online, 'offline', retain=True)
        self._client.on_connect = self._on_mqtt_connect
        self._client.on_disconnect = self._on_mqtt_disconnect
        self._client.on_subscribe = self._on_mqtt_subscribe
        self._client.on_message = self._on_mqtt_message

        cmd_period = 1.0 / max(1.0, float(self.get_parameter('cmd_timer_hz').value))
        state_period = 1.0 / max(0.2, float(self.get_parameter('state_hz').value))
        self.create_timer(cmd_period, self._publish_cmd)
        self.create_timer(state_period, self._publish_state)
        self.create_timer(30.0, self._ensure_connected)
        self.create_timer(5.0, self._check_slam_map_health)

        try:
            self._client.connect_async(self._host, self._port, keepalive=30)
            self._client.loop_start()
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().error(f'MQTT 连接启动失败: {exc}')
        self.get_logger().info(
            f'mqtt_bridge 启动：{self._host}:{self._port} device={self._device}')
        self._lan_ip_cache = ''
        self._lan_ip_at = 0.0

    def _publish_online_heartbeat(self) -> None:
        """周期性重申在线状态（retained），顺便自愈被旧会话 LWT 覆盖的情况。"""
        if not self._connected:
            return
        value = 'offline' if (self._shutting_down or
                              _system_shutting_down()) else 'online'
        try:
            self._client.publish(self._topic_online, value, retain=True, qos=1)
        except Exception:                        # noqa: BLE001
            pass

    # ---------------- MQTT 回调 ----------------
    def _on_mqtt_connect(self, client, _userdata, _flags, reason_code, _props=None):
        self._connected = reason_code == 0
        if self._connected:
            if self._shutting_down or _system_shutting_down():
                # 关机过程中被 systemd 重启：绝不能报 online，
                # 否则手机端会闪一下"设备已重新上线"
                client.publish(self._topic_online, 'offline', retain=True)
                self.get_logger().warn('系统正在关机：本次连接只发 offline')
            else:
                client.publish(self._topic_online, 'online', retain=True)
            client.subscribe(self._topic_cmd, qos=0)
            client.subscribe(self._topic_goal, qos=1)
            client.subscribe(self._topic_nav, qos=1)
            client.subscribe(self._topic_preview_cmd, qos=0)
            client.subscribe(self._topic_power, qos=1)
            client.subscribe(self._topic_lmap_plain, qos=1)
            client.subscribe(self._topic_map_edit, qos=1)
            if not self._map_published:
                self._publish_map()
            self.get_logger().info('MQTT 已连接')
            # 在线状态心跳：retained 消息可能被旧会话的 LWT 覆盖，
            # 每 20 秒重申一次，最多 20 秒内自愈
            try:
                if self._online_timer is None:
                    self._online_timer = self.create_timer(
                        20.0, self._publish_online_heartbeat)
            except Exception as exc:             # noqa: BLE001
                self.get_logger().warn(f'在线心跳定时器创建失败: {exc}')
        else:
            self.get_logger().error(f'MQTT 连接失败: {reason_code}')

    def _resubscribe(self) -> None:
        """幂等重订阅全部上行话题（防静默掉订阅）。"""
        try:
            for topic, qos in ((self._topic_cmd, 0), (self._topic_goal, 1),
                               (self._topic_nav, 1),
                               (self._topic_preview_cmd, 0),
                               (self._topic_power, 1),
                               (self._topic_lmap_plain, 1),
                               (self._topic_map_edit, 1)):
                self._client.subscribe(topic, qos=qos)
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().warn(f'重订阅失败: {exc}',
                                   throttle_duration_sec=30.0)

    def _on_mqtt_subscribe(self, _client, _userdata, mid, reason_codes,
                           _props=None):
        bad = [str(rc) for rc in (reason_codes or []) if int(getattr(rc, 'value', rc)) >= 128]
        if bad:
            self.get_logger().error(
                f'MQTT 订阅被拒绝 mid={mid}: {bad}（该话题将收不到消息）')

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
        if msg.topic == self._topic_lmap_plain:
            self._on_lmap_plain(msg)
            return
        if msg.topic == self._topic_map_edit:
            self._on_map_edit(msg)
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

    def _on_filtered_vel(self, msg):
        self._last_filtered = time.monotonic()

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
            if 'x' in data and 'y' in data:
                self._leg_x = float(data['x'])
                self._leg_y = float(data['y'])
                self._leg_at = time.monotonic()
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
        self._publish_map_file(self._map_path())

    def _publish_map_file(self, yaml_path) -> None:
        """把指定地图文件压缩发到云端（retained）。切图时先发新图，
        手机不用等导航栈重启完（2026-11-20 用户反馈"等云端更新太久"）。"""
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
            self._published_map_size = (width, height)
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
        # 2026-11-20 现场事故：手机在 test1 地图上点了 6 个航点，但车端已切到
        # 1A_map_door2，坐标按旧图解释 → 目标落在墙里 → 原地左右打转。
        # 目标里带 map 字段时校验：跟"已加载地图"不一致就拒绝，让用户重点。
        try:
            declared = str(obj.get('map') or '').strip()
        except AttributeError:
            declared = ''
        active = self._current_map_name()
        if declared and active and declared != active:
            self.get_logger().warn(
                f'目标来自旧地图 {declared}，当前已加载 {active}，拒绝执行')
            self._publish_result('goal', False,
                                 f'地图已切换为 {active}，请重新点航点')
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
        # 2026-11-19：把收到的 nav 动作记一行日志，否则"手机点了没反应"时
        # 完全看不出是消息没到、还是队列卡住（现场排查很吃亏）。
        self.get_logger().info(f'收到 nav 动作: {action}')
        # 2026-11-19：relocalize 是手机端的周期请求（每 12 秒一次）。
        # 导航栈没起来时 /localization/reseed 一直 not ready，这些请求会被
        # 反复 appendleft 堆在队首，把后面的 mapping_stop 堵住 →
        # 车端永远停在建图栈，手机就"一直重新定位"（16:47 现场）。
        # 这里只保留最新的一条，避免堆积。
        if action == 'relocalize':
            self._pending = type(self._pending)(
                it for it in self._pending
                if not (it[0] == 'nav' and isinstance(it[1], dict)
                        and it[1].get('action') == 'relocalize'))
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
        # 先立刻报离线（快于 keepalive/LWT 的 30 秒），再动手关机，
        # 这样手机端是"点完就离线"，不会等 MQTT 超时
        self._shutting_down = True
        try:
            self._client.publish(self._topic_online, 'offline', retain=True, qos=1)
        except Exception:                        # noqa: BLE001
            pass
        try:
            subprocess.run(cmd, timeout=10, check=False)
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().error(f'执行电源指令失败: {exc}')
            self._shutting_down = False

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
        # 2026-11-19：_busy 一旦被某个没回调的 future 卡住，
        # 后面所有动作（包括手机的模式切换）都会静默排队，现象是"点了没反应"。
        # 这里给 _busy 加 30 秒兜底，超时强制解锁并告警。
        if self._busy:
            now_b = time.monotonic()
            if not self._busy_since:
                self._busy_since = now_b
            elif now_b - self._busy_since > 30.0:
                self.get_logger().warn(
                    '动作队列卡住超过 30 秒，强制解锁并继续处理')
                self._busy = False
                self._busy_since = 0.0
            # 2026-11-19：模式切换/手动授权与 ROS 服务是否 busy 无关，
            # 必须能插队处理，否则一个卡住的导航请求会把手机指令全堵死
            # （现场表现：点「授权」一直"已请求授权"、切模式没反应）。
            # 模式切换（起/停栈）优先级最高：它们才是"把服务拉起来"的动作，
            # 必须先跑，否则 relocalize/stop 之类会在服务不在时无限重排。
            def _prio(item):
                k, pl = item
                if k == 'nav' and isinstance(pl, dict):
                    a = pl.get('action')
                    if a in ('mapping_start', 'mapping_stop', 'mapping_restart'):
                        return 0
                    if a in ('relocalize', 'stop', 'resume'):
                        return 2
                return 1
            nav_idx = None
            for i, item in enumerate(self._pending):
                if item[0] != 'nav':
                    continue
                if nav_idx is None or _prio(item) < _prio(self._pending[nav_idx]):
                    nav_idx = i
            if nav_idx is None:
                return
            kind, payload = self._pending[nav_idx]
            del self._pending[nav_idx]
            self.get_logger().info('busy 中优先处理 nav 动作')
        else:
            self._busy_since = 0.0
            if not self._pending:
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
            if action == 'delete_map':
                self._delete_map(str(obj.get('map') or ''))
                return
            if action in ('mapping_start', 'mapping_stop', 'mapping_restart'):
                self._run_service_action(action)
                return
            if action == 'mode':
                # 2026-11-24 用户要求：切模式时**先把现有模式清理干净**
                # （停掉所有不属于目标模式的栈，确认真的停了再起目标栈），
                # 开机只跑 fwr-base（硬件层）→ 手动模式。
                self._switch_mode(str(obj.get('mode') or 'manual'))
                return
            if action == 'relocalize':
                client = self._reseed_client
                if not client.service_is_ready():
                    # 导航栈没起来时这个服务本来就不存在。手机每 12 秒会重发，
                    # 这里直接丢弃并记一行，绝不能 appendleft（会饿死模式切换）。
                    self.get_logger().warn(
                        '重新定位服务未就绪（导航栈未运行？），已忽略本次请求',
                        throttle_duration_sec=10.0)
                    # 必须回一条失败结果，否则手机端只能等到 15 次超时才停
                    self._publish_result('nav', False, '导航栈未就绪，请稍候',
                                         action='relocalize')
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
        """设置运动门。

        2026-11-20 现场（开机后手机点授权一直"请求中"）：
        旧实现是 `wait_for_service(0.3)` + `_pending.appendleft` 重排。
        开机时 base_bridge 还没起来（它在导航栈里，要 60~90 s），
        于是每 0.1 s 的 drain 都阻塞 0.3 s 再重排 → **把执行器饿死**，
        `wait_for_service` 再也看不到图变化，永远 False，自锁。
        现在改成**非阻塞**：服务没就绪就只记下目标门位，由 1 Hz 定时器
        `_apply_pending_gate` 在服务出现后补发，并回结果给手机。
        """
        self._desired_gate = (bool(motion), bool(manual), bool(mixed))
        if not self._param_client.service_is_ready():
            self.get_logger().warn(
                '运动门等待 /base_bridge（车端仍在启动），稍后自动重试',
                throttle_duration_sec=5.0)
            return
        self._send_gate()

    def _send_gate(self) -> None:
        if self._desired_gate is None:
            return
        motion, manual, mixed = self._desired_gate
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
        self._desired_gate = None
        future = self._param_client.call_async(req)
        future.add_done_callback(self._on_gate_done)
        self.get_logger().info(
            f'运动门设置 motion={motion} manual={manual} mixed={mixed}')

    def _on_gate_done(self, future) -> None:
        ok = False
        try:
            res = future.result()
            ok = all(r.successful for r in getattr(res, 'results', []))
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().warn(f'设置运动门失败: {exc}')
        if ok:
            self.get_logger().info('运动门已生效')
            self._publish_result('nav', True, '手动授权已开启', action='manual')
        else:
            self.get_logger().error('运动门设置失败')
            self._publish_result('nav', False, '运动门设置失败', action='manual')

    def _apply_pending_gate(self) -> None:
        """1 Hz：有未生效的门位且服务已就绪时补发。"""
        if self._desired_gate is None:
            return
        if self._param_client.service_is_ready():
            self._send_gate()

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

    def _active_stack(self) -> str:
        """当前实际在跑哪个栈：'map' / 'nav' / ''（都停着）。"""
        try:
            out = subprocess.run(
                ['systemctl', 'is-active', 'fwr-map.service'],
                capture_output=True, text=True, timeout=4).stdout.strip()
            if out == 'active':
                return 'map'
            out = subprocess.run(
                ['systemctl', 'is-active', 'fwr-nav.service'],
                capture_output=True, text=True, timeout=4).stdout.strip()
            if out == 'active':
                return 'nav'
        except Exception:                       # noqa: BLE001
            pass
        return ''

    def _systemctl(self, verb: str, unit: str, timeout: float = 60) -> None:
        """sudo -n systemctl <verb> <unit>，失败只记录不抛。"""
        try:
            subprocess.run(['sudo', '-n', 'systemctl', verb, unit],
                           timeout=timeout, check=False)
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().error(f'systemctl {verb} {unit} 出错: {exc}')

    def _service_state(self, unit: str) -> str:
        try:
            return subprocess.run(['systemctl', 'is-active', unit],
                                  capture_output=True, text=True,
                                  timeout=5).stdout.strip()
        except Exception:                        # noqa: BLE001
            return ''

    def _wait_service_state(self, unit: str, wanted, timeout: float) -> bool:
        """轮询直到 unit 处于 wanted 中的任一状态（确认真的停了/起了）。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            st = self._service_state(unit)
            if st in wanted:
                return True
            time.sleep(0.5)
        self.get_logger().warn(
            f'{unit} 等待 {wanted} 超时，当前={self._service_state(unit) or "unknown"}')
        return False

    # ---- 2026-11-24：模式栈（按需启停）----
    # 开机只有 fwr-base（硬件层，手动模式可用）；导航/建图/跟随按需启动。
    # 切模式的第一条规则是**先清理干净**：停掉所有不属于目标模式的栈，
    # 等到 systemd 真的报 inactive 再启动目标栈（否则两套节点同抢
    # /scan、/tf、串口 → "建图和导航打架"）。
    MODE_UNITS = {
        'manual': (),
        'nav': ('fwr-nav.service',),
        'map': ('fwr-map.service',),
        'follow': ('fwr-vision.service', 'fwr-follow.service'),
    }
    ALL_MODE_UNITS = ('fwr-nav.service', 'fwr-map.service',
                      'fwr-vision.service', 'fwr-follow.service')
    BASE_UNIT = 'fwr-base.service'

    def _switch_mode(self, target: str) -> None:
        import threading
        target = (target or '').strip().lower()
        if target not in self.MODE_UNITS:
            # 2026-11-24：**不能回落成手动了**。云端/页面任一环丢了 mode 字段时，
            # 旧写法会把"切导航"执行成"切手动"——现场就是反复「已切到手动模式」
            # 而导航栈永远起不来。未知模式一律忽略，保持现状。
            self.get_logger().warn(
                f'忽略未知模式 {target!r}（只认 {sorted(self.MODE_UNITS)}）')
            return
        self._nav = 'unknown'
        with self._mode_lock:
            if self._mode_running:
                self._mode_pending_mode = target
                self.get_logger().warn(
                    f'mode={target} 排队（{self._mode_running_action} 正在切换）')
                return
            self._mode_running = True
            self._mode_running_action = f'mode:{target}'
        want = self.MODE_UNITS[target]

        def worker():
            try:
                # 1) 硬件层必须活着（手动模式全靠它）
                if self._service_state(self.BASE_UNIT) != 'active':
                    self.get_logger().warn('fwr-base 未运行，先拉起硬件层')
                    self._systemctl('reset-failed', self.BASE_UNIT, timeout=20)
                    self._systemctl('start', self.BASE_UNIT, timeout=90)
                    if not self._wait_service_state(self.BASE_UNIT, ('active',),
                                                   timeout=45.0):
                        self.get_logger().error('fwr-base 起不来，模式切换中止')
                        self._publish_result('nav', False, '硬件层启动失败',
                                             action='mode')
                        return
                # 2) 先清理：把不属于目标模式的栈全部停掉
                for unit in self.ALL_MODE_UNITS:
                    if unit in want:
                        continue
                    if self._service_state(unit) in ('inactive', 'failed', None):
                        continue
                    self.get_logger().warn(f'切到 {target}：先停 {unit}')
                    self._systemctl('stop', unit, timeout=90)
                    if not self._wait_service_state(unit, ('inactive', 'failed'),
                                                   timeout=45.0):
                        self.get_logger().error(
                            f'{unit} 未能停止，中止切换到 {target}')
                        self._publish_result('nav', False,
                                             f'{unit} 未能停止，已取消', action='mode')
                        return
                if target == 'manual':
                    self._mapping_mode = False
                    self._publish_result('nav', True, '手动模式（已清理其它模式）',
                                         action='mode')
                    self.get_logger().warn('已切到手动模式（只保留硬件层）')
                    return
                # 3) 再启动目标模式的栈
                for unit in want:
                    self._systemctl('reset-failed', unit, timeout=20)
                    self._systemctl('start', unit, timeout=120)
                    if not self._wait_service_state(unit, ('active',), timeout=45.0):
                        self.get_logger().warn(f'{unit} 未 active，重试一次')
                        self._systemctl('reset-failed', unit, timeout=20)
                        self._systemctl('start', unit, timeout=120)
                        if not self._wait_service_state(unit, ('active',),
                                                       timeout=45.0):
                            self.get_logger().error(f'{unit} 启动失败')
                            self._publish_result('nav', False,
                                                 f'{unit} 启动失败', action='mode')
                            return
                # 4) 模式标志
                if target == 'map':
                    self._mapping_mode = True
                    self._mapping_grace_until = time.monotonic() + 25.0
                    self._live_map = None
                    self._live_map_dirty = False
                    self._publish_live_map_reset()
                    self._mapping_suppress_until = 0.0
                else:
                    self._mapping_mode = False
                    self._mapping_suppress_until = time.monotonic() + 20.0
                self.get_logger().warn(f'已切到 {target} 模式（其它模式已清理）')
                self._publish_result('nav', True, f'{target} 模式已就绪',
                                     action='mode')
            except Exception as exc:             # noqa: BLE001
                self.get_logger().error(f'模式切换到 {target} 失败: {exc}')
                self._publish_result('nav', False, f'切换失败: {exc}', action='mode')
            finally:
                with self._mode_lock:
                    self._mode_running = False
                    self._mode_running_action = ''
                    pending_mode = getattr(self, '_mode_pending_mode', '')
                    self._mode_pending_mode = ''
                if pending_mode and pending_mode != target:
                    self.get_logger().warn(f'执行排队中的模式切换: {pending_mode}')
                    self._switch_mode(pending_mode)

        threading.Thread(target=worker, daemon=True).start()

    def _run_service_action(self, action: str) -> None:
        """建图/导航栈切换：后台线程执行 systemctl。

        2026-11-19：**串行化**。原来 page 连点两下「建图探索/导航」会并发启动两个
        systemctl 序列，互相 stop 对方的栈，最后落在哪个栈全看时序
        （实测 10:08:37 两次 mapping_start + 10:08:57 mapping_stop 并发，
        最终停在建图栈 → 手机导航模式一直重新定位）。
        现在：执行中只记"最后一个想要的"，完成后再执行；目标栈已经是当前栈就跳过。
        """
        import threading
        target = 'map' if action in ('mapping_start', 'mapping_restart') else 'nav'
        self._nav = 'unknown'   # 栈要动，先别让手机以为导航就绪
        with self._mode_lock:
            if self._mode_running:
                self._mode_pending = action
                self.get_logger().warn(
                    f'{action} 排队（{self._mode_running_action} 正在切换栈）')
                return
            if action in ('mapping_start', 'mapping_stop'):
                active = self._active_stack()
                if active == target:
                    self.get_logger().info(
                        f'{action} 忽略：栈已经是 {target} 模式')
                    self._publish_result('nav', True, f'已经是{("建图" if target == "map" else "导航")}模式',
                                         action=action)
                    return
            self._mode_running = True
            self._mode_running_action = action
        # 每个动作 = (先停哪个栈, 再起哪个栈)。
        # 用户要求：**必须先确认另一边的栈真的停了，再启动这一边**，
        # 否则两套节点会同时占用 /scan、/tf、/cmd_vel，出现"建图和导航打架"。
        plans = {
            'mapping_start': ('fwr-nav.service', 'fwr-map.service'),
            'mapping_stop': ('fwr-map.service', 'fwr-nav.service'),
            'mapping_restart': ('fwr-map.service', 'fwr-map.service'),
            # 2026-11-20 bug：原来是 ('fwr-map', 'fwr-nav') + start fwr-nav ——
            # 导航栈本来就在跑时 systemd 的 start 是空操作，于是"换地图"只写了
            # 状态文件、map_server 还挂着旧图，手机换新图后定位仍按旧图算
            # （15:31 现场：选了 test1，map_server 仍是 1A_map_door2 928x605）。
            # nav 换图必须**先停自己再起**，才能让 run_navigation.sh 重读
            # ~/.config/fwr/last_navigation_map。
            'nav_restart': ('fwr-nav.service', 'fwr-nav.service'),
        }
        plan = plans.get(action)
        if plan is None:
            return
        # 切栈前先把排队里的"轻量动作"清掉：它们在旧栈上执行毫无意义，
        # 还会在切栈完成后继续和新栈打架。
        self._pending = type(self._pending)(
            it for it in self._pending
            if not (it[0] == 'nav' and isinstance(it[1], dict)
                    and it[1].get('action') in ('relocalize', 'stop', 'resume')))
        if action in ('mapping_start', 'mapping_restart'):
            self._mapping_mode = True
            self._mapping_grace_until = time.monotonic() + 25.0
            # 清空缓存并通知手机丢弃旧地图：SLAM 重启后新地图是空白的，
            # 手机如果继续保留旧图，看起来就像"重新建图没生效"（11-19 现场）。
            self._live_map = None
            self._live_map_dirty = False
            self._publish_live_map_reset()
        elif action in ('mapping_stop', 'nav_restart'):
            self._mapping_mode = False
            self._mapping_suppress_until = time.monotonic() + 20.0

        def worker():
            before_map_msgs = self._nav_map_msgs
            stop_svc, start_svc = plan
            self.get_logger().warn(f'{action}: 先停 {stop_svc}')
            try:
                # 1) 停：等它真的变成 inactive/failed 才继续
                self._systemctl('stop', stop_svc, timeout=60)
                stopped = self._wait_service_state(stop_svc,
                                                   ('inactive', 'failed'),
                                                   timeout=30.0)
                if not stopped:
                    self.get_logger().error(
                        f'{action}: {stop_svc} 30 秒内没停下，放弃启动 {start_svc}')
                    self._publish_result(
                        'nav', False,
                        f'{stop_svc} 未能停止，已取消启动 {start_svc}',
                        action=action)
                    return
                self.get_logger().info(f'{action}: {stop_svc} 已停止')
                time.sleep(2.0)                  # 给 DDS 一点时间回收节点
                # 2) 起：先清 failed 状态，起不来重试一次
                self._systemctl('reset-failed', start_svc, timeout=20)
                self._systemctl('start', start_svc, timeout=90)
                if not self._wait_service_state(start_svc, ('active',),
                                                timeout=25.0):
                    self.get_logger().warn(
                        f'{action}: {start_svc} 未 active，清 failed 后重试一次')
                    self._systemctl('reset-failed', start_svc, timeout=20)
                    self._systemctl('start', start_svc, timeout=90)
                    if not self._wait_service_state(start_svc, ('active',),
                                                    timeout=25.0):
                        self.get_logger().error(
                            f'{action}: {start_svc} 启动失败')
                        self._publish_result(
                            'nav', False, f'{start_svc} 启动失败', action=action)
                        return
                self.get_logger().warn(f'{action} 完成（{stop_svc} 已停，{start_svc} 已起）')
                if action in ('nav_restart', 'mapping_stop'):
                    # 2026-11-20：确认地图真的换过来了。以前 systemctl start
                    # 在栈已运行时是空操作，状态文件写了却仍按旧图定位，
                    # 页面还显示"切换成功"。这里等 /map 真刷新，等不到就报错。
                    deadline = time.monotonic() + 60.0

                    def map_refreshed() -> bool:
                        # 2026-11-20：原来要求"至少 2 帧新 /map"，可 map_server
                        # 只锁存发布 1 帧（map_publish_period=0）→ 换图成功也判失败。
                        if self._nav_map_msgs > before_map_msgs:
                            return True
                        return (self._pending_map_size is not None and
                                self._nav_map_size == self._pending_map_size)

                    while time.monotonic() < deadline and not map_refreshed():
                        time.sleep(0.5)
                    if not map_refreshed():
                        self.get_logger().error(
                            f'{action}: 60 秒内 /map 没有刷新，地图可能没换成功'
                            f'（当前 {self._nav_map_size}）')
                        # 预发的新图不算数：把"真正在用的旧图"发回去
                        try:
                            self._publish_map_file(self._active_map_path)
                        except Exception:            # noqa: BLE001
                            pass
                        self._pending_map_path = ''
                        self._publish_result(
                            'nav', False, '地图没有刷新，请重试', action=action)
                    elif self._pending_map_path:
                        # 地图真换了才算"已切换"（页面靠这个字段弹提示）
                        self._active_map_path = self._pending_map_path
                        self._pending_map_path = ''
                        self._pending_map_size = None
                        self._map_name = Path(self._active_map_path).stem
                        self.get_logger().warn(
                            f'地图切换完成（已生效）: {self._map_name}')
                        # 真正加载完才通知手机"已加载"
                        self._publish_result('nav', True, self._map_name,
                                             action='set_map_done')
                    self._map_name_at = 0.0
                    self._publish_map()
                self._publish_result('nav', True, f'{action} 完成',
                                     action=action)
            except Exception as exc:             # noqa: BLE001
                self.get_logger().error(f'{action} 失败: {exc}')
                self._publish_result('nav', False, f'{action} 失败: {exc}',
                                     action=action)
            finally:
                with self._mode_lock:
                    self._mode_running = False
                    self._mode_running_action = ''
                    nxt = self._mode_pending
                    self._mode_pending = ''
                if nxt:
                    self.get_logger().warn(f'执行排队中的 {nxt}')
                    self._run_service_action(nxt)

        threading.Thread(target=worker, daemon=True).start()

    def _check_slam_map_health(self) -> None:
        """建图中 SLAM 地图持续 0×0 → 自动重启建图栈（黑屏自愈）。

        2026-11-19 现场：双雷达在建图栈刚起时集体 USB 超时死掉，
        slam_toolbox 的首帧队列被丢弃，之后地图一直停在 0×0，
        手机端就是全黑。重启整栈即可恢复（实测 173×193）。
        """
        if not self._mapping_mode:
            self._empty_map_since = 0.0
            return
        now = time.monotonic()
        if self._slam_map_w > 0:
            self._empty_map_since = 0.0
            return
        if not self._empty_map_since:
            self._empty_map_since = now
            return
        if now - self._empty_map_since < 90.0:
            return
        if now - self._last_map_recover < 600.0:
            return
        self._last_map_recover = now
        self._empty_map_since = 0.0
        self.get_logger().warn('SLAM 地图持续 90 秒为 0×0，自动重启建图栈')
        self._run_service_action('mapping_restart')

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

    MAPS_ROOT = '/home/manbaout/Company_Project/Four-wheel-robot/ros2_ws/src/fwr_bringup/maps'

    def _map_dirs(self):
        """返回 [(目录, 来源标记)]。phone = 手机上编辑/保存的图，robot = SLAM 存的图。"""
        from pathlib import Path
        root = Path(self.MAPS_ROOT)
        return [(root, 'robot'),
                (root / 'phone', 'phone'),
                (Path.home() / 'robot_maps', 'robot')]

    def _map_files(self):
        """列出所有可用地图：[(名字, 路径, 来源)]。"""
        out = []
        for d, source in self._map_dirs():
            if not d.is_dir():
                continue
            for f in sorted(d.glob('*.yaml')):
                out.append((f.name.replace('.yaml', ''), str(f), source))
        return out

    def _on_map_edit(self, msg) -> None:
        """手机端编辑后的地图：写成 maps/<name>.pgm + .yaml，并设为导航地图。

        手机只传栅格字节（0=占用 205=未知 254=空闲），这里负责补 PGM 头、
        生成 YAML 元数据、更新 last_navigation_map 指针，然后回报结果。
        """
        import base64
        import gzip
        from pathlib import Path
        try:
            obj = json.loads(msg.payload.decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            return
        try:
            raw = base64.b64decode(obj.get('pgm') or '')
            if obj.get('gz'):
                raw = gzip.decompress(raw)
        except Exception as exc:                 # noqa: BLE001
            self._publish_result('nav', False, f'地图解码失败: {exc}',
                                 action='map_edit')
            return
        w = int(obj.get('width') or 0)
        h = int(obj.get('height') or 0)
        res = float(obj.get('resolution') or 0.05)
        if w <= 0 or h <= 0 or len(raw) != w * h:
            self._publish_result(
                'nav', False,
                f'地图尺寸不匹配：{w}x{h} 应为 {w * h} 字节，实收 {len(raw)}',
                action='map_edit')
            return
        origin = obj.get('origin') or {}
        ox = float(origin.get('x') or 0.0)
        oy = float(origin.get('y') or 0.0)
        oyaw = float(origin.get('yaw') or 0.0)
        name = str(obj.get('name') or '').strip()
        if not name:
            name = time.strftime('edit_%Y%m%d_%H%M%S')
        name = Path(name).name.replace('.yaml', '').replace('.pgm', '') or \
            time.strftime('edit_%Y%m%d_%H%M%S')
        # 2026-11-20 用户要求：手机上编辑/保存的图单独放 maps/phone/，
        # 跟 SLAM 存下来的图分开，方便辨认和整体删除。
        maps_dir = Path(self.MAPS_ROOT) / 'phone'
        pgm_path = maps_dir / f'{name}.pgm'
        yaml_path = maps_dir / f'{name}.yaml'
        try:
            maps_dir.mkdir(parents=True, exist_ok=True)
            pgm_path.write_bytes(b'P5\n%d %d\n255\n' % (w, h) + raw)
            yaml_path.write_text(
                f'image: {name}.pgm\n'
                f'resolution: {res:.6f}\n'
                f'origin: [{ox:.6f}, {oy:.6f}, {oyaw:.6f}]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.196\n',
                encoding='utf-8')
            state = Path.home() / '.config/fwr/last_navigation_map'
            state.parent.mkdir(parents=True, exist_ok=True)
            state.write_text(str(yaml_path), encoding='utf-8')
        except OSError as exc:
            self.get_logger().error(f'保存编辑地图失败: {exc}')
            self._publish_result('nav', False, f'保存失败: {exc}',
                                 action='map_edit')
            return
        self.get_logger().warn(
            f'编辑地图已保存: {pgm_path.name} {w}x{h} @{res:.3f}m')
        self._map_name = name
        self._map_name_at = 0.0
        self._publish_result('nav', True, f'{name}（{w}×{h}）',
                             action='map_edit')
        self._publish_map_list()

    def _publish_map_list(self) -> None:
        maps = [{'name': n, 'path': pth, 'source': src}
                for n, pth, src in self._map_files()]
        if not self._connected:
            return
        self._client.publish(self._topic_maps,
                             json.dumps({'maps': maps}, ensure_ascii=False),
                             qos=1)
        self.get_logger().info(f'地图列表已上报：{len(maps)} 个')

    def _delete_map(self, path: str) -> None:
        """删除一张地图（.yaml + .pgm），拒绝删当前正在用的图和越界路径。"""
        from pathlib import Path
        p = Path(path or '')
        try:
            p = p.resolve()
        except OSError:
            self._publish_result('nav', False, '路径无效', action='delete_map')
            return
        allowed = False
        for d, _src in self._map_dirs():
            try:
                if p.parent == d.resolve():
                    allowed = True
                    break
            except OSError:
                continue
        if not allowed or p.suffix != '.yaml' or not p.is_file():
            self._publish_result('nav', False, '不能删除这个文件',
                                 action='delete_map')
            return
        state = Path.home() / '.config/fwr/last_navigation_map'
        try:
            current = state.read_text(encoding='utf-8').strip()
        except OSError:
            current = ''
        if current and Path(current).resolve() == p:
            self._publish_result('nav', False, '这是当前使用的地图，先换一张再删',
                                 action='delete_map')
            return
        removed = []
        for suffix in ('.yaml', '.pgm', '.json'):
            f = p.with_suffix(suffix)
            if f.is_file():
                try:
                    f.unlink()
                    removed.append(f.name)
                except OSError as exc:
                    self.get_logger().warn(f'删除 {f} 失败: {exc}')
        if not removed:
            self._publish_result('nav', False, '删除失败', action='delete_map')
            return
        self.get_logger().warn(f'已删除地图: {", ".join(removed)}')
        self._publish_result('nav', True, '已删除 ' + p.stem, action='delete_map')
        self._publish_map_list()

    def _apply_map_choice(self, path: str) -> None:
        from pathlib import Path
        if not path:
            return
        p = Path(path)
        if not p.is_file():
            self.get_logger().warn(f'地图不存在: {path}')
            return
        # 2026-11-20：选的就是当前已加载的图 → 不折腾（否则白重启一次导航栈）
        try:
            if (self._active_map_path and
                    Path(self._active_map_path).resolve() == p and
                    not self._pending_map_path):
                self.get_logger().info(f'已经是当前地图 {p.name}，不重启')
                # 已经是这张图 → 直接算"已加载"（不需要重启，也不用等）
                self._publish_result('nav', True, p.stem,
                                     action='set_map_done')
                return
        except OSError:
            pass
        self._pending_map_path = str(p)
        # 栈马上要重启：先把导航状态标回未知，手机据此判断"还没就绪"，
        # 不会在重启窗口里狂点重新定位
        self._nav = 'unknown'
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
        # 2026-11-20 用户反馈：刚点切换就提示"地图已加载"，其实图还没出来。
        # 这里只回报"已受理/切换中"，真正加载完成由 worker 在 /map 刷新后
        # 用 action='set_map_done' 回报。
        self.get_logger().info(f'导航地图已切换为 {p.name}，重启导航栈')
        # 2026-11-20：新图文件已经在磁盘上，先直接发到云端（几十毫秒），
        # 手机 1~2 秒就能看到新图，不用等导航栈 40~60 秒的重启。
        try:
            self._publish_map_file(str(p))
            self._pending_map_size = self._published_map_size
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().warn(f'切图时预发地图失败: {exc}')
        self._publish_result('nav', True, p.stem, action='set_map_ack')
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
        self._nav_map_msgs += 1

    def _on_live_map_fallback(self, msg) -> None:
        """没有 /slam_map 时（旧启动方式）才使用 /map，避免混入其它节点的地图。"""
        if self._slam_map_seen:
            return
        self._on_live_map(msg, from_slam=False)

    def _on_live_map(self, msg, from_slam: bool = True) -> None:
        """SLAM 地图：压缩后低频上传，供手机建图模式显示。"""
        if from_slam:
            self._slam_map_w = int(msg.info.width)
        if not self._mapping_mode:
            return
        if time.monotonic() < self._mapping_grace_until:
            return          # 切换宽限期：忽略旧导航栈残留的 /map
        if time.monotonic() < self._mapping_suppress_until:
            return          # 刚停止建图：抑制残留的实时地图
        now = time.monotonic()
        if now - self._live_map_at < 1.0:
            return
        self._last_occ = msg
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
            if self._live_map_plain:
                if from_slam:
                    self._slam_map_seen = True
                self._live_map = self._plain_live_map(msg)
                self._live_map_dirty = True
                self._live_map_at = now
                return
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

    def _on_lmap_plain(self, msg) -> None:
        try:
            self._live_map_plain = bool(
                json.loads(msg.payload.decode()).get('plain'))
        except Exception:                        # noqa: BLE001
            pass
        self.get_logger().info(
            f'实时地图格式：{"未压缩" if self._live_map_plain else "gzip"}')
        # 立刻用最后一帧按新格式重发，避免"车不动 → 没有新地图帧 → 页面一直黑"
        if self._last_occ is not None:
            self._live_map_at = 0.0
            self._on_live_map(self._last_occ)

    def _plain_live_map(self, msg) -> dict:
        """未压缩版本：0=占用 160=未知 254=空闲，大图按 2 倍下采样。

        ⚠️ 行序必须翻转：OccupancyGrid 的 data 是"从下往上"，
        页面按图片行序（从上往下）绘制，并用 y = height - (y-origin)/res 换算，
        不翻转会让地图上下镜像 → 雷达点和墙对不上（现场"错位"）。
        """
        w, h = int(msg.info.width), int(msg.info.height)
        data = msg.data
        # 只在真的很大时才降采样（0.1 m），普通房间保持 0.05 m 全分辨率
        step = 1 if w * h <= 260000 else 2
        ow, oh = max(1, w // step), max(1, h // step)
        out = bytearray(ow * oh)
        for oy in range(oh):
            for ox in range(ow):
                any_occ = False
                all_free = True
                for dy in range(step):
                    grid_y = h - 1 - (oy * step + dy)   # 下往上 → 上往下
                    if grid_y < 0 or grid_y >= h:
                        all_free = False
                        continue
                    base = grid_y * w
                    for dx in range(step):
                        idx = base + ox * step + dx
                        if idx >= len(data):
                            all_free = False
                            continue
                        v = data[idx]
                        if v >= 50:                     # 占用
                            any_occ = True
                            all_free = False
                        elif v != 0:                    # -1 未知
                            all_free = False
                out[oy * ow + ox] = 0 if any_occ else (254 if all_free else 160)
        return {
            'resolution': float(msg.info.resolution) * step,
            'origin': [float(msg.info.origin.position.x),
                       float(msg.info.origin.position.y), 0.0],
            'width': ow,
            'height': oh,
            'pgm': base64.b64encode(bytes(out)).decode('ascii'),
            'plain': True,
        }

    def _sync_mapping_mode(self) -> None:
        """每 5 秒对一次账：当前跑的是哪个模式的栈。

        2026-11-24：不再只有 map/nav 两态 —— 开机只跑 fwr-base（手动模式），
        导航/建图/跟随按需启动。这里用 systemctl 的真实状态算模式，
        别用"我以为切到哪了"，否则手机显示和车端会长期不一致。
        """
        def _act(unit: str) -> bool:
            try:
                out = subprocess.run(['systemctl', 'is-active', unit],
                                     capture_output=True, text=True, timeout=3)
                return out.stdout.strip() == 'active'
            except Exception:                    # noqa: BLE001
                return False

        try:
            self._mapping_mode = _act('fwr-map')
            if self._mapping_mode:
                self._current_mode = 'map'
            elif _act('fwr-nav'):
                self._current_mode = 'nav'
            elif _act('fwr-follow') or _act('fwr-vision'):
                self._current_mode = 'follow'
            else:
                self._current_mode = 'manual'
        except Exception:                        # noqa: BLE001
            pass

    def _init_active_map(self) -> None:
        """进程启动时把状态文件里的地图记为"已加载"（栈正是按它起的）。"""
        from pathlib import Path
        try:
            state = Path.home() / '.config/fwr/last_navigation_map'
            if state.is_file():
                self._active_map_path = state.read_text(encoding='utf-8').strip()
                self._map_name = Path(self._active_map_path).stem
        except OSError:
            pass

    def _current_map_name(self) -> str:
        from pathlib import Path
        # 只报"真正加载"的那张图，别把刚选中还没生效的报出去
        if self._active_map_path:
            return Path(self._active_map_path).stem
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

    def _publish_live_map_reset(self) -> None:
        if not self._connected:
            return
        self._client.publish(self._topic_live_map,
                             json.dumps({'reset': True}), qos=0)

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

    def _on_scan_merged(self, msg) -> None:
        """双雷达 360° 合并扫描（/scan_merged，base_link 系，已去除车身自击）。"""
        self._merged_at = time.monotonic()
        self._handle_scan(msg, merged=True)

    def _on_scan_front(self, msg) -> None:
        self._front_at = time.monotonic()
        # 单雷达兜底（合并扫描 1 秒内没数据时才用）
        if time.monotonic() - self._merged_at < 1.0:
            return
        self._handle_scan(msg, merged=False)

    def _on_scan_rear(self, msg) -> None:
        self._rear_at = time.monotonic()

    def _handle_scan(self, msg, merged: bool) -> None:
        """把激光点转到地图坐标、抽稀后发布（默认 5 Hz / 360 点）。"""
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
            wrapped = math.atan2(math.sin(ang), math.cos(ang))
            # 单雷达时屏蔽车尾 ±50°（车体自击）；双雷达合并圈是 360°，不遮
            if not merged and abs(wrapped) <= self._rear_mask_rad:
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
        # 2026-11-23：失败结果以前只发给手机、车上不留痕；现场排查
        # "一直说没就绪"时完全看不到真实原因（残差过大/贴墙率不足…）。
        if not ok:
            self.get_logger().warn(
                'nav 动作失败: %s → %s' % (self._last_nav_action, reason),
                throttle_duration_sec=5.0)
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

    def _lan_ip(self) -> str:
        """本机 LAN 地址（30 s 缓存，取不到返回空串）。

        UDP `connect` 不发包，只让内核选路由 —— 比 gethostbyname 准
        （后者在 /etc/hosts 里可能解析到 127.0.1.1）。
        """
        now = time.monotonic()
        if self._lan_ip_cache and now - self._lan_ip_at < 30.0:
            return self._lan_ip_cache
        ip = ''
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.settimeout(0.2)
                sock.connect(('8.8.8.8', 80))
                ip = sock.getsockname()[0]
            finally:
                sock.close()
        except Exception:                    # noqa: BLE001
            ip = ''
        if not ip or ip.startswith('127.'):
            try:
                ip = socket.gethostbyname(socket.gethostname())
            except Exception:                # noqa: BLE001
                ip = ''
        if ip.startswith('127.'):
            ip = ''
        self._lan_ip_cache = ip
        self._lan_ip_at = now
        return ip

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
                                      self._leg, self._legs,
                                      self._front_at, self._rear_at,
                                      mode=self._current_mode)
        # build_state_payload 返回的是 JSON bytes，这里解回 dict 才能加字段
        try:
            body = json.loads(payload.decode('utf-8'))
        except (ValueError, AttributeError):
            body = {}
        # 2026-11-23：上报本机 IP。云端 /api/devices/{id}/media 的取值顺序是
        # JC_MEDIA_HOSTS → 状态里的 lan_ip。机器人 IP 由 DHCP 分配，
        # 今天从 .20 变成 .24 就导致手机黑屏、跟随页全废（要手工改云端配置）。
        # 带上 lan_ip，云端清空 JC_MEDIA_HOSTS 后就能自动跟上换网/换 IP。
        body['lan_ip'] = self._lan_ip()
        # 安全链是否通畅（过滤后速度 2 s 内有帧）。建图模式没有这条链，给 True。
        body['chain'] = (True if self._mapping_mode else
                         (time.monotonic() - self._last_filtered) < 2.0)
        # 当前正在前往的目标坐标（3 秒内有效），手机用它算"第几号航点"
        if (self._leg_x is not None and
                time.monotonic() - self._leg_at < 3.0):
            body['leg_x'] = round(self._leg_x, 3)
            body['leg_y'] = round(self._leg_y, 3)
        self._client.publish(
            self._topic_state,
            json.dumps(body, separators=(',', ':')).encode('utf-8'), qos=0)

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
            # 2026-11-19：出现过"cmd 还能收、nav 却收不到"的静默掉订阅
            # （QoS1 订阅莫名失效，SUBACK 也没记录），手机端点「授权」就
            # 一直卡在"已请求授权"。这里每 30 秒幂等重订阅一次，自愈。
            self._resubscribe()
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
