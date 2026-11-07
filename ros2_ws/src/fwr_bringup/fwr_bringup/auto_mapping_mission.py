#!/usr/bin/env python3
"""Auto-mapping mission finalizer.

Watches the viewpoint planner's completion state, closes the motion gates,
stops the mapping mission, and saves the SLAM map through /mapping/save_slam.

This is deliberately a NEW node.  It does not modify manual mapping,
navigation launch files, behaviour trees, or frozen navigation parameters.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger


class CompletionGate:
    """Pure completion debounce: a transient 'done' must not trigger a save."""

    def __init__(self, hold_s: float) -> None:
        self.hold_s = max(0.0, float(hold_s))
        self._since: float | None = None

    def reset(self) -> None:
        self._since = None

    def update(self, state: str, now: float) -> bool:
        if not state.startswith('done'):
            self.reset()
            return False
        if self._since is None:
            self._since = now
        return now - self._since >= self.hold_s


class AutoMappingMission(Node):
    def __init__(self) -> None:
        super().__init__('auto_mapping_mission')
        self.declare_parameter('completion_hold_s', 3.0)
        self.declare_parameter('save_on_done', True)
        self.declare_parameter('close_gate_on_done', True)
        # 0 = no cap.  A small-area live trial sets e.g. 90 s; this is a hard
        # safety stop independent of coverage completion.
        self.declare_parameter('max_runtime_s', 0.0)
        self._gate = CompletionGate(
            float(self.get_parameter('completion_hold_s').value))
        self._active_since: float | None = None
        self._max_runtime = float(self.get_parameter('max_runtime_s').value)
        self._finished = False
        self._saving = False
        self._state = 'waiting'

        self._state_pub = self.create_publisher(String, '/mapping/mission_state', 10)
        self._run_pub = self.create_publisher(
            Bool, '/mapping/run', QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            String, '/viewpoint_planner/state', self._on_planner_state, 10)
        self.create_subscription(
            Int32, '/base/fault', self._on_fault, 10)
        self._save_client = self.create_client(Trigger, '/mapping/save_slam')
        self._param_client = self.create_client(
            SetParameters, '/base_bridge/set_parameters')
        self._save_future = None
        self.create_timer(0.5, self._tick)
        self._publish('waiting')
        self.get_logger().info(
            'auto mapping mission finalizer ready (hold %.1f s, save=%s)' % (
                self._gate.hold_s, self.get_parameter('save_on_done').value))

    def _publish(self, text: str) -> None:
        if text == self._state:
            return
        self._state = text
        self._state_pub.publish(String(data=text))

    def _on_fault(self, msg: Int32) -> None:
        if msg.data == 0 or self._finished:
            return
        self.get_logger().error('base fault %d → hard stop and save' % msg.data)
        self._finish('base_fault %d' % msg.data)

    def _on_planner_state(self, msg: String) -> None:
        state = msg.data
        now = self.get_clock().now().nanoseconds / 1e9
        if (self._active_since is None and
                state.startswith(('bootstrap', 'going', 'batch', 'rolling',
                                  'chain', 'viewpoint', 'frontier', 'perimeter',
                                  'expand', 'dispatch', 'dwell'))):
            self._active_since = now
            self.get_logger().info('auto mapping motion window started')
        if not self._gate.update(state, now):
            return
        if not self._finished:
            self._finish(state)

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if (not self._finished and self._active_since is not None and
                self._max_runtime > 0.0 and
                now - self._active_since >= self._max_runtime):
            self._finish('time_limit %.0fs' % self._max_runtime)
            return
        if self._save_future is not None and self._save_future.done():
            future = self._save_future
            self._save_future = None
            self._saving = False
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001
                self._publish('save_failed: %s' % exc)
                self.get_logger().error('map save failed: %s' % exc)
                return
            if response is not None and response.success:
                self._publish('saved: %s' % response.message)
                self.get_logger().info('auto mapping saved: %s' % response.message)
            else:
                message = response.message if response is not None else 'no response'
                self._publish('save_failed: %s' % message)
                self.get_logger().error('auto mapping save rejected: %s' % message)

    def _close_gate(self) -> None:
        self._run_pub.publish(Bool(data=False))
        if not self._param_client.service_is_ready():
            self.get_logger().warn('base_bridge parameter service not ready; gate unchanged')
            return
        request = SetParameters.Request()
        for name in ('motion_enabled', 'manual_enabled'):
            parameter = RclParameter()
            parameter.name = name
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = False
            request.parameters.append(parameter)
        self._param_client.call_async(request)

    def _finish(self, planner_state: str) -> None:
        self._finished = True
        self._publish('finishing: %s' % planner_state)
        self.get_logger().info('auto mapping complete: %s' % planner_state)
        if bool(self.get_parameter('close_gate_on_done').value):
            self._close_gate()
        if not bool(self.get_parameter('save_on_done').value):
            self._publish('done (save disabled)')
            return
        if not self._save_client.service_is_ready():
            self._publish('save_failed: /mapping/save_slam unavailable')
            self.get_logger().error(
                '/mapping/save_slam unavailable; is real_mapping.launch.py running?')
            return
        self._saving = True
        self._save_future = self._save_client.call_async(Trigger.Request())


def main() -> None:
    rclpy.init()
    node = AutoMappingMission()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
