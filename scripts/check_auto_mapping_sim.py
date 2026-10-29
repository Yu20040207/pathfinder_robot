#!/usr/bin/env python3
"""Boot the isolated headless mapping stack and verify its disarmed state."""
import argparse
import math
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_srvs.srv import SetBool, Trigger
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64MultiArray, String


def is_armed_motion_state(state):
    return state in ('navigating', 'reversing')


def reverse_acceptance(raw_commands, unchecked_commands, safe_commands,
                       distance_m, max_segment_m, stopped):
    return (raw_commands > 0 and unchecked_commands > 0 and safe_commands > 0 and
            0 < distance_m <= .30 and 0 < max_segment_m <= .105 and stopped)


def front_stop_acceptance(injected_at, last_forward_at, now):
    return (injected_at is not None and last_forward_at is not None and
            -.2 <= last_forward_at - injected_at <= .5 and
            now - last_forward_at >= .8)


def rear_block_acceptance(raw_commands, safe_commands, distance_m):
    return raw_commands > 0 and safe_commands == 0 and distance_m == 0


class Probe(Node):
    def __init__(self):
        super().__init__('auto_mapping_sim_probe')
        self.known = 0
        self.nav_succeeded = False
        self.map_msg = None
        self.last_map_at = None
        self.scans = 0
        self.last_scan_at = None
        self.last_odom_at = None
        self.nonzero_unarmed = False
        self.motion_commands = 0
        self.reverse_commands = 0
        self.reverse_raw_commands = 0
        self.reverse_unchecked_commands = 0
        self.reverse_filtered_commands = 0
        self.reverse_filtered_messages = 0
        self.reverse_safe_commands = 0
        self.reverse_distance = 0.
        self.max_reverse_segment = 0.
        self.plan_count = 0
        self.last_plan = None
        self.goals = []
        self.last_motion_at = None
        self.last_forward_motion_at = None
        self.xy_start = None
        self.xy_latest = None
        self.xy_trace = []
        self.distance_traveled = 0.
        self.explore_start = self.create_client(Trigger, '/explore/start')
        self.arm_auto = self.create_client(Trigger, '/base/arm_auto')
        self.explore_stop = self.create_client(Trigger, '/explore/stop')
        self.scan_switch = self.create_client(SetBool, '/sim/scan_enabled')
        self.front_switch = self.create_client(SetBool, '/sim/front_blocked')
        self.rear_switch = self.create_client(SetBool, '/sim/rear_blocked')
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.state = None
        self.state_transitions = []
        self.armed = False
        self.seen_reversing = False
        self.action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self._nav_status, 10)
        self.nav_state = self.create_client(GetState, '/bt_navigator/get_state')
        self.create_subscription(OccupancyGrid, '/map', self._map, 10)
        self.create_subscription(Path, '/plan', self._plan, 10)
        self.create_subscription(PoseStamped, '/explore/goal', self._goal, 10)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_sim_enforced', self._safe, 10)
        self.create_subscription(Twist, '/cmd_vel_recovery_raw',
                                 lambda msg: setattr(self, 'reverse_raw_commands',
                                                     self.reverse_raw_commands + int(msg.linear.x < -1e-4)), 10)
        self.create_subscription(Twist, '/cmd_vel_auto_unchecked',
                                 lambda msg: setattr(self, 'reverse_unchecked_commands',
                                                     self.reverse_unchecked_commands + int(msg.linear.x < -1e-4)), 10)
        self.create_subscription(Twist, '/cmd_vel_auto_reverse_filtered', self._reverse_filtered, 10)
        self.create_subscription(Twist, '/cmd_vel_auto_safe',
                                 lambda msg: setattr(self, 'reverse_safe_commands',
                                                     self.reverse_safe_commands + int(msg.linear.x < -1e-4)), 10)
        self.create_subscription(String, '/explore/state', self._state, 10)
        self.create_subscription(Bool, '/base/auto_armed',
                                 lambda msg: setattr(self, 'armed', msg.data), 10)
        self.create_subscription(Float64MultiArray, '/sim/recovery_budget',
                                 lambda msg: setattr(self, 'max_reverse_segment',
                                                     max(self.max_reverse_segment, msg.data[0])), 10)

    def _map(self, msg):
        self.map_msg = msg
        self.known = max(self.known, sum(value >= 0 for value in msg.data))
        self.last_map_at = time.monotonic()

    def _nav_status(self, msg):
        self.nav_succeeded = self.nav_succeeded or any(
            entry.status == GoalStatus.STATUS_SUCCEEDED for entry in msg.status_list)

    def _plan(self, msg):
        self.plan_count += 1
        self.last_plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _goal(self, msg):
        self.goals.append((msg.pose.position.x, msg.pose.position.y))


    def _scan(self, msg):
        self.scans += 1
        self.last_scan_at = time.monotonic()

    def _odom(self, msg):
        self.last_odom_at = time.monotonic()
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.xy_start is None:
            self.xy_start = xy
        if self.xy_latest is not None:
            step = math.dist(self.xy_latest, xy)
            if step < .1:
                self.distance_traveled += step
                if self.state == 'reversing':
                    self.reverse_distance += step
        self.xy_latest = xy
        self.xy_trace.append(xy)

    def _safe(self, msg):
        if msg.linear.x < -1e-4:
            self.reverse_commands += 1
        if abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6:
            self.motion_commands += 1
            self.last_motion_at = time.monotonic()
            if self.state == 'navigating':
                self.last_forward_motion_at = self.last_motion_at
        if not is_armed_motion_state(self.state) and (abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6):
            self.nonzero_unarmed = True

    def _reverse_filtered(self, msg):
        self.reverse_filtered_messages += 1
        self.reverse_filtered_commands += int(msg.linear.x < -1e-4)

    def _state(self, msg):
        if msg.data != self.state:
            now = time.monotonic()
            self.state_transitions.append((round(now, 3), msg.data,
                                           *(round(now - stamp, 3) if stamp else None
                                             for stamp in (self.last_map_at, self.last_scan_at,
                                                           self.last_odom_at))))
        self.state = msg.data
        if self.state == 'reversing':
            self.seen_reversing = True


def move_static_obstacle(env, x, y):
    """Move a preloaded Gazebo obstacle without pausing sensors for model creation."""
    result = subprocess.run(
        ['ign', 'service', '-s', '/world/corridor/set_pose',
         '--reqtype', 'ignition.msgs.Pose',
         '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000',
         '--req', f'name: "path_obstacle", position: {{x: {x:.3f}, y: {y:.3f}, z: 0.25}}, orientation: {{w: 1}}'],
        env=env, capture_output=True, text=True, timeout=8, check=False)
    return result.returncode == 0 and 'data: true' in result.stdout, result.stdout + result.stderr


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=75.)
    parser.add_argument('--domain', type=int, default=97)
    parser.add_argument('--exercise-seconds', type=float, default=0.)
    parser.add_argument('--inject-scan-loss', action='store_true')
    parser.add_argument('--inject-front-obstacle', action='store_true')
    parser.add_argument('--inject-after-distance', type=float, default=.05)
    parser.add_argument('--spawn-path-obstacle', action='store_true')
    parser.add_argument('--spawn-on-long-goal', action='store_true')
    parser.add_argument('--obstacle-lateral', type=float, default=0.)
    parser.add_argument('--hold-obstacle-seconds', type=float, default=2.)
    parser.add_argument('--require-reverse', action='store_true')
    parser.add_argument('--block-rear-during-front', action='store_true')
    args = parser.parse_args()
    if sum((args.inject_scan_loss, args.inject_front_obstacle,
            args.spawn_path_obstacle)) > 1:
        parser.error('select one fault injection per run')
    if args.require_reverse and not args.inject_front_obstacle:
        parser.error('--require-reverse requires --inject-front-obstacle')
    if (args.inject_scan_loss or args.inject_front_obstacle or args.spawn_path_obstacle) and args.exercise_seconds <= 0:
        parser.error('fault injection requires --exercise-seconds > 0')
    if args.block_rear_during_front and not args.inject_front_obstacle:
        parser.error('--block-rear-during-front requires --inject-front-obstacle')
    if args.block_rear_during_front and args.require_reverse:
        parser.error('rear block and reverse-required scenarios are mutually exclusive')
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(args.domain)
    env['IGN_PARTITION'] = f'fwr_auto_sim_{args.domain}'
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    with tempfile.NamedTemporaryFile(prefix='fwr_auto_sim_', suffix='.log', delete=False) as log:
        log_path = log.name
        process = subprocess.Popen(['ros2', 'launch', 'fwr_bringup',
                                    'corridor_sim.launch.py', 'headless:=true',
                                    'auto_mapping:=true', 'rviz:=false'],
                                   env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
            if process.poll() is not None:
                break
            if (node.known > 100 and node.scans > 5 and node.action.server_is_ready() and
                    node.explore_start.service_is_ready() and node.arm_auto.service_is_ready() and
                    node.state == 'stopped'):
                break
        if args.exercise_seconds and process.poll() is None and node.explore_start.service_is_ready():
            nav_deadline = time.monotonic() + 20.
            nav_active = False
            while time.monotonic() < nav_deadline and not nav_active:
                if node.nav_state.service_is_ready():
                    state_future = node.nav_state.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(node, state_future, timeout_sec=1.)
                    nav_active = bool(state_future.done() and state_future.result() and
                                      state_future.result().current_state.label == 'active')
                rclpy.spin_once(node, timeout_sec=.2)
            print(f'nav_lifecycle_active={nav_active}')
            armed = node.arm_auto.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, armed, timeout_sec=3.)
            print(f'auto_arm={armed.result().success if armed.done() and armed.result() else False} '
                  f'reason={armed.result().message if armed.done() and armed.result() else "no response"}')
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=.1)
            start_result = None
            start_deadline = time.monotonic() + 5.
            while time.monotonic() < start_deadline:
                request = node.explore_start.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, request, timeout_sec=1.)
                start_result = request.result() if request.done() else None
                if start_result and start_result.success:
                    break
                rclpy.spin_once(node, timeout_sec=.2)
            print(f'explore_start={start_result.success if start_result else False} '
                  f'reason={start_result.message if start_result else "no response"} '
                  f'map_age={time.monotonic() - node.last_map_at if node.last_map_at else float("inf"):.3f}')
            if not start_result or not start_result.success:
                print(f'preflight_failed log={log_path}')
                return 1
            exercise_deadline = time.monotonic() + args.exercise_seconds
            loss_injected = False
            loss_passed = False
            obstacle_injected = False
            obstacle_passed = False
            obstacle_injected_at = None
            spawned = False
            spawn_at = None
            plans_at_spawn = None
            trace_at_spawn = None
            path_obstacle_passed = False
            while time.monotonic() < exercise_deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.1)
                if (args.inject_scan_loss and not loss_injected and
                        node.motion_commands > 5 and node.xy_start is not None and
                        node.xy_latest is not None and
                        ((node.xy_latest[0]-node.xy_start[0])**2 +
                         (node.xy_latest[1]-node.xy_start[1])**2)**.5 > .05 and
                        node.scan_switch.service_is_ready()):
                    change = node.scan_switch.call_async(SetBool.Request(data=False))
                    rclpy.spin_until_future_complete(node, change, timeout_sec=2.)
                    loss_injected = bool(change.done() and change.result() and change.result().success)
                    print(f'scan_loss_injected={loss_injected}')
                    break
                if (args.inject_front_obstacle and not obstacle_injected and
                        node.distance_traveled > args.inject_after_distance and
                        node.front_switch.service_is_ready()):
                    if args.block_rear_during_front:
                        change = node.rear_switch.call_async(SetBool.Request(data=True))
                        rclpy.spin_until_future_complete(node, change, timeout_sec=2.)
                        if not (change.done() and change.result() and change.result().success):
                            print('rear_obstacle_injection_failed')
                            return 1
                    change = node.front_switch.call_async(SetBool.Request(data=True))
                    rclpy.spin_until_future_complete(node, change, timeout_sec=2.)
                    obstacle_injected = bool(change.done() and change.result() and change.result().success)
                    obstacle_injected_at = time.monotonic() if obstacle_injected else None
                    print(f'front_obstacle_injected={obstacle_injected} '
                          f'distance_traveled={node.distance_traveled:.3f}m')
                    break
                if (args.spawn_path_obstacle and not spawned and
                        node.xy_start is not None and node.xy_latest is not None and
                        ((args.spawn_on_long_goal and len(node.goals) >= 2 and
                          math.dist(node.xy_latest, node.goals[-1]) > 1.4) or
                         (not args.spawn_on_long_goal and
                          ((node.xy_latest[0]-node.xy_start[0])**2 +
                           (node.xy_latest[1]-node.xy_start[1])**2)**.5 > .05))):
                    if args.spawn_on_long_goal:
                        target = node.goals[-1]
                        dx, dy = target[0]-node.xy_latest[0], target[1]-node.xy_latest[1]
                        scale = min(.9 / max(math.hypot(dx, dy), 1e-9), .6)
                        spawn_at = (node.xy_latest[0] + dx*scale,
                                    node.xy_latest[1] + dy*scale + args.obstacle_lateral)
                    else:
                        spawn_at = (node.xy_latest[0] + .95,
                                    node.xy_latest[1] + args.obstacle_lateral)
                    spawned, detail = move_static_obstacle(env, *spawn_at)
                    plans_at_spawn = node.plan_count
                    trace_at_spawn = len(node.xy_trace)
                    print(f'path_obstacle_spawned={spawned} at={spawn_at} response={detail.strip()[:200]}')
                    break
            if args.inject_scan_loss and loss_injected:
                loss_deadline = time.monotonic() + 4.
                while time.monotonic() < loss_deadline and (node.armed or node.state != 'stopped'):
                    rclpy.spin_once(node, timeout_sec=.1)
                count_at_stop = node.motion_commands
                change = node.scan_switch.call_async(SetBool.Request(data=True))
                rclpy.spin_until_future_complete(node, change, timeout_sec=2.)
                for _ in range(15):
                    rclpy.spin_once(node, timeout_sec=.1)
                loss_passed = (not node.armed and node.state == 'stopped' and
                               node.motion_commands == count_at_stop)
                print(f'scan_loss_latched={loss_passed}')
            if args.inject_front_obstacle and obstacle_injected:
                observe_until = time.monotonic() + args.hold_obstacle_seconds
                while time.monotonic() < observe_until:
                    rclpy.spin_once(node, timeout_sec=.1)
                armed_while_blocked = node.armed
                state_while_blocked = node.state
                stop_latency = (node.last_forward_motion_at - obstacle_injected_at
                                if node.last_forward_motion_at and obstacle_injected_at else float('nan'))
                obstacle_passed = front_stop_acceptance(
                    obstacle_injected_at, node.last_forward_motion_at, time.monotonic())
                if args.hold_obstacle_seconds <= 3.0 and not args.require_reverse:
                    obstacle_passed = obstacle_passed and armed_while_blocked and state_while_blocked == 'navigating'
                if args.hold_obstacle_seconds >= 20.0 and not args.require_reverse:
                    obstacle_passed = obstacle_passed and not armed_while_blocked and state_while_blocked == 'stopped'
                change = node.front_switch.call_async(SetBool.Request(data=False))
                rclpy.spin_until_future_complete(node, change, timeout_sec=2.)
                if args.block_rear_during_front:
                    change = node.rear_switch.call_async(SetBool.Request(data=False))
                    rclpy.spin_until_future_complete(node, change, timeout_sec=2.)
                resumed_after_clear = None
                if args.hold_obstacle_seconds <= 3.0:
                    motion_at_clear = node.motion_commands
                    resume_deadline = time.monotonic() + 4.
                    while time.monotonic() < resume_deadline and node.motion_commands == motion_at_clear:
                        rclpy.spin_once(node, timeout_sec=.1)
                    resumed_after_clear = node.motion_commands > motion_at_clear
                    obstacle_passed = obstacle_passed and resumed_after_clear
                print(f'front_obstacle_stopped={obstacle_passed} seen_reversing={node.seen_reversing} '
                      f'reverse_commands={node.reverse_commands} '
                      f'reverse_raw={node.reverse_raw_commands} '
                      f'reverse_unchecked={node.reverse_unchecked_commands} '
                      f'reverse_filtered={node.reverse_filtered_commands}/{node.reverse_filtered_messages} '
                      f'reverse_safe={node.reverse_safe_commands} '
                      f'reverse_distance={node.reverse_distance:.3f}m '
                      f'armed_while_blocked={armed_while_blocked} state_while_blocked={state_while_blocked} '
                      f'resumed_after_clear={resumed_after_clear} '
                      f'stop_latency={stop_latency:.3f}s')
            if args.spawn_path_obstacle and spawned:
                observe_until = time.monotonic() + args.hold_obstacle_seconds
                while time.monotonic() < observe_until and node.armed:
                    rclpy.spin_once(node, timeout_sec=.1)
                plan_y_span = (max(y for _, y in node.last_plan) - min(y for _, y in node.last_plan)
                               if node.last_plan else float('nan'))
                traversed = node.xy_trace[trace_at_spawn:]
                closest_center = min((math.dist(p, spawn_at) for p in traversed), default=float('nan'))
                farthest_x = max((x for x, _ in traversed), default=float('nan'))
                # The conservative circular envelope contains the 0.60 x 0.43 m
                # planner footprint, plus 0.18 m obstacle radius and a 5 cm margin.
                clearance_limit = math.hypot(.30, .215) + .18 + .05
                path_obstacle_passed = (farthest_x > spawn_at[0] + .2 and
                                        closest_center > clearance_limit and
                                        node.plan_count > plans_at_spawn)
                print(f'path_obstacle_observed armed={node.armed} state={node.state} '
                      f'new_plans={node.plan_count - plans_at_spawn} '
                      f'latest_plan_y_span={plan_y_span:.3f} xy={node.xy_latest} '
                      f'max_x={farthest_x:.3f} min_center_distance={closest_center:.3f} '
                      f'clearance_limit={clearance_limit:.3f} reroute_passed={path_obstacle_passed}')
            if node.explore_stop.service_is_ready():
                stop = node.explore_stop.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, stop, timeout_sec=3.)
            stop_deadline = time.monotonic() + 6.
            while time.monotonic() < stop_deadline and node.state != 'stopped':
                rclpy.spin_once(node, timeout_sec=.1)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=.1)
        passed = (process.poll() is None and node.known > 100 and node.scans > 5
                  and node.action.server_is_ready() and node.state == 'stopped'
                  and not node.nonzero_unarmed)
        if args.exercise_seconds:
            moved = (node.xy_start is not None and node.xy_latest is not None and
                     ((node.xy_latest[0]-node.xy_start[0])**2 + (node.xy_latest[1]-node.xy_start[1])**2)**.5 > .03)
            passed = passed and moved and node.motion_commands > 0
            if args.inject_scan_loss:
                passed = passed and loss_injected and loss_passed
            if args.inject_front_obstacle:
                passed = passed and obstacle_injected and obstacle_passed
            if args.require_reverse:
                reverse_passed = reverse_acceptance(
                    node.reverse_raw_commands, node.reverse_unchecked_commands,
                    node.reverse_commands, node.reverse_distance, node.max_reverse_segment,
                    node.state == 'stopped' and
                    (node.last_motion_at is None or time.monotonic() - node.last_motion_at >= .3))
                passed = passed and reverse_passed
                print(f'reverse_accepted={reverse_passed} '
                      f'reverse_commands={node.reverse_commands} '
                      f'reverse_distance={node.reverse_distance:.3f}m '
                      f'max_segment={node.max_reverse_segment:.3f}m')
            if args.block_rear_during_front:
                rear_block_passed = rear_block_acceptance(
                    node.reverse_raw_commands, node.reverse_safe_commands,
                    node.reverse_distance)
                passed = passed and rear_block_passed
                print(f'rear_block_accepted={rear_block_passed}')
            if args.spawn_path_obstacle:
                passed = passed and spawned
                if args.hold_obstacle_seconds >= 40.:
                    passed = passed and path_obstacle_passed
        print(f'known={node.known} scans={node.scans} nav_action={node.action.server_is_ready()} '
              f'state={node.state} unarmed_motion={node.nonzero_unarmed} '
              f'motion_commands={node.motion_commands} xy={node.xy_latest} log={log_path}')
        if args.inject_front_obstacle or args.spawn_path_obstacle:
            print(f'state_transitions={node.state_transitions[-20:]}')
        return 0 if passed else 1
    finally:
        print('cleanup: destroy probe', flush=True)
        node.destroy_node()
        print('cleanup: shutdown rclpy', flush=True)
        if rclpy.ok():
            rclpy.shutdown()
        print('cleanup: stop simulator', flush=True)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        print('cleanup: done', flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
