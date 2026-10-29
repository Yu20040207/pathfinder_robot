#!/usr/bin/env python3
"""Exercise a fixed Nav2 goal and a movable obstacle through the auto safety chain."""
import argparse
import math
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger
from fwr_interfaces.srv import StartFixedGoal
from fwr_bringup.fixed_goal_core import validate_fixed_goal

from check_auto_mapping_sim import Probe, move_static_obstacle


def spin_until(node, predicate, deadline):
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        if predicate():
            return True
    return False


def chassis_obstacle_clearance(pose, obstacle, half_length=.26, half_width=.175,
                               obstacle_radius=.18):
    """Distance between the Gazebo chassis box and cylindrical obstacle in XY."""
    x, y, yaw = pose
    dx, dy = obstacle[0] - x, obstacle[1] - y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    box_distance = math.hypot(max(abs(local_x) - half_length, 0.),
                              max(abs(local_y) - half_width, 0.))
    return box_distance - obstacle_radius


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--domain', type=int, default=141)
    parser.add_argument('--timeout', type=float, default=100.)
    parser.add_argument('--goal-x', type=float, default=2.3)
    parser.add_argument('--no-obstacle', action='store_true')
    parser.add_argument('--obstacle-x', type=float, default=1.5)
    parser.add_argument('--obstacle-y', type=float, default=0.)
    args = parser.parse_args()
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(args.domain)
    env['IGN_PARTITION'] = f'fwr_fixed_reroute_{args.domain}'
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    with tempfile.NamedTemporaryFile(prefix='fwr_fixed_reroute_', suffix='.log', delete=False) as log:
        log_path = log.name
        process = subprocess.Popen(
            ['ros2', 'launch', 'fwr_bringup', 'corridor_sim.launch.py',
             'headless:=true', 'auto_mapping:=true', 'auto_explorer:=true',
             'auto_saved_map:=true', 'rviz:=false'],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Probe()
    commanded = {'linear': 0, 'angular': 0, 'safe_angular': 0, 'safe_max_z': 0.}
    safe_samples = []
    poses = []
    world_trace = []
    def observe_odom(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        poses.append((round(msg.pose.pose.position.x, 2),
                      round(msg.pose.pose.position.y, 2), round(yaw, 2)))
    def observe_command(msg):
        commanded['linear'] += int(abs(msg.linear.x) > 1e-3)
        commanded['angular'] += int(abs(msg.angular.z) > 1e-3)
    def observe_safe(msg):
        commanded['safe_angular'] += int(abs(msg.angular.z) > 1e-3)
        commanded['safe_max_z'] = max(commanded['safe_max_z'], abs(msg.angular.z))
        if abs(msg.angular.z) > .01 and len(safe_samples) < 20:
            safe_samples.append((round(msg.linear.x, 3), round(msg.angular.z, 3)))
    def observe_world(msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'fwr_chassis':
                p = transform.transform.translation
                q = transform.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                world_trace.append((p.x, p.y, yaw))
                break
    node.create_subscription(Odometry, '/odom', observe_odom, 10)
    node.create_subscription(Twist, '/cmd_vel_nav', observe_command, 10)
    node.create_subscription(Twist, '/cmd_vel_sim_enforced', observe_safe, 10)
    node.create_subscription(TFMessage, '/gazebo_world_poses', observe_world, 10)
    node.create_subscription(
        OccupancyGrid, '/map', node._map,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    fixed_client = node.create_client(StartFixedGoal, '/explore/start_fixed_goal')
    try:
        deadline = time.monotonic() + args.timeout
        ready = spin_until(node, lambda: (process.poll() is None and node.known > 100 and
                           node.scans > 5 and fixed_client.service_is_ready() and
                           node.arm_auto.service_is_ready() and node.state == 'stopped'), deadline)
        if not ready:
            print(f'stack_not_ready log={log_path}')
            return 1
        nav_active = False
        while time.monotonic() < deadline and not nav_active:
            if node.nav_state.service_is_ready():
                current = node.nav_state.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, current, timeout_sec=1.)
                nav_active = bool(current.done() and current.result() and
                                  current.result().current_state.label == 'active')
            rclpy.spin_once(node, timeout_sec=.2)
        if not nav_active:
            print(f'nav_lifecycle_inactive log={log_path}')
            return 1
        # Arm only after the simulated scan and odometry are current.
        arm_result = None
        for _ in range(10):
            arm = node.arm_auto.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, arm, timeout_sec=1.)
            arm_result = arm.result() if arm.done() else None
            if arm_result and arm_result.success:
                break
            rclpy.spin_once(node, timeout_sec=.2)
        if not arm_result or not arm_result.success:
            print(f'arm_failed reason={arm_result.message if arm_result else "no response"} log={log_path}')
            return 1
        if not spin_until(node, lambda: node.armed, time.monotonic() + 2.):
            print(f'arm_status_not_published log={log_path}')
            return 1
        valid, why = validate_fixed_goal(node.map_msg, 0., 0., args.goal_x, 0., 0., 3.)
        print(f'goal_map_check valid={valid} reason={why}', flush=True)
        sent = fixed_client.call_async(StartFixedGoal.Request(x=args.goal_x, y=0., yaw=0.))
        rclpy.spin_until_future_complete(node, sent, timeout_sec=5.)
        result = sent.result() if sent.done() else None
        if not result or not result.accepted:
            print(f'goal_rejected reason={result.reason if result else "timeout"} log={log_path}')
            return 1
        if not spin_until(node, lambda: node.state == 'navigating', time.monotonic() + 5.):
            print(f'no_navigating_state log={log_path}')
            return 1
        started = spin_until(node, lambda: (node.xy_latest is not None and node.xy_start is not None and
                   math.dist(node.xy_latest, node.xy_start) > .05), time.monotonic() + 25.)
        if not started:
            print(f'no_initial_motion xy={node.xy_latest} plans={node.plan_count} log={log_path}')
            return 1
        if args.no_obstacle:
            spin_until(node, lambda: node.state == 'stopped', deadline)
            passed = (node.xy_latest is not None and
                      node.xy_latest[0] > .5 and node.nav_succeeded and
                      node.state == 'stopped' and not node.nonzero_unarmed)
            print(f'fixed_goal_passed={passed} xy={node.xy_latest} '
                  f'plans={node.plan_count} nav_succeeded={node.nav_succeeded} '
                  f'state={node.state} log={log_path}')
            return 0 if passed else 1
        spawned, detail = move_static_obstacle(env, args.obstacle_x, args.obstacle_y)
        if not spawned:
            print(f'obstacle_move_failed response={detail[:200]} log={log_path}')
            return 1
        plan_before = node.plan_count
        print(f'obstacle_at=({args.obstacle_x},{args.obstacle_y}) xy_at_injection={node.xy_latest} plans={plan_before}')
        crossed = spin_until(node, lambda: (node.xy_latest is not None and
                   node.xy_latest[0] > args.obstacle_x + .2), deadline)
        if crossed:
            spin_until(node, lambda: node.state == 'stopped' and not node.armed, deadline)
        trace = world_trace
        closest = min((chassis_obstacle_clearance(p, (args.obstacle_x, args.obstacle_y))
                       for p in trace), default=float('nan'))
        clearance = .04
        passed = (crossed and node.xy_latest is not None and node.xy_latest[0] > args.obstacle_x + .2 and
                  closest > clearance and node.plan_count > plan_before and
                  node.nav_succeeded and not node.nonzero_unarmed and
                  not node.armed and node.state == 'stopped')
        print(f'reroute_passed={passed} xy={node.xy_latest} world_pose={world_trace[-1] if world_trace else None} '
              f'min_chassis_clearance={closest:.3f} '
              f'limit={clearance:.3f} new_plans={node.plan_count-plan_before} '
              f'nav_succeeded={node.nav_succeeded} armed={node.armed} '
              f'state={node.state} log={log_path}')
        print(f'commanded={commanded} safe_samples={safe_samples} recent_poses={poses[-10:]} '
              f'last_plan={node.last_plan[::max(1, len(node.last_plan)//12)] if node.last_plan else None}')
        return 0 if passed else 1
    finally:
        if node.explore_stop.service_is_ready():
            node.explore_stop.call_async(Trigger.Request())
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)


if __name__ == '__main__':
    raise SystemExit(main())
