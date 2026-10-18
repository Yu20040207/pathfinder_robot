from fwr_interfaces.action import TurnRelative


def test_turn_action_has_explicit_modes_and_result():
    goal = TurnRelative.Goal()
    assert goal.PIVOT == 1 and goal.ARC == 2
    goal.angle_deg = -5.0
    goal.mode = goal.PIVOT
    result = TurnRelative.Result()
    result.success = False
    result.reason = 'imu_stale'
    result.actual_angle_deg = -1.0
    assert (goal.angle_deg, result.reason) == (-5.0, 'imu_stale')


def test_turn_wait_yields_to_rclpy_executor_without_asyncio_loop():
    from rclpy.task import Task
    from fwr_bringup.base_bridge import BaseBridgeNode

    class TimerHost:
        callback = None
        destroyed = False

        def create_timer(self, _, callback):
            self.callback = callback
            return self

        def destroy_timer(self, _):
            self.destroyed = True

    host = TimerHost()
    task = Task(BaseBridgeNode._sleep_ros(host, .05))
    task()
    assert not task.done()
    host.callback()
    task()
    assert task.done()
    assert task.result() is None
    assert host.destroyed


def test_nav_goal_status_tracks_active_goal_during_command_pause():
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from fwr_bringup.base_bridge import BaseBridgeNode

    class Host:
        _nav_active = {'pose': False, 'through': False}

    host = Host()
    msg = GoalStatusArray()
    item = GoalStatus()
    item.status = GoalStatus.STATUS_EXECUTING
    msg.status_list = [item]
    BaseBridgeNode._on_nav_status(host, 'pose', msg)
    assert host._nav_active['pose']
    item.status = GoalStatus.STATUS_SUCCEEDED
    BaseBridgeNode._on_nav_status(host, 'pose', msg)
    assert not host._nav_active['pose']


def test_turn_goal_rejected_when_nav_goal_active_without_recent_cmd_vel():
    from rclpy.action import GoalResponse
    from fwr_bringup.base_bridge import BaseBridgeNode

    class Host:
        _nav_active = {'pose': True, 'through': False}

        def _sensors_ready(self, _):
            return True

    class Request:
        mode = 2
        angle_deg = 5.0

    assert BaseBridgeNode._turn_goal(Host(), Request()) == GoalResponse.REJECT


def test_navigation_status_subscriptions_request_retained_status():
    from pathlib import Path
    source = (Path(__file__).resolve().parents[1] / 'fwr_bringup' / 'base_bridge.py').read_text()
    assert 'qos_profile_action_status_default' in source
    assert "lambda msg: self._on_nav_status('pose', msg),\n                                 qos_profile_action_status_default" in source
    assert "lambda msg: self._on_nav_status('through', msg),\n                                 qos_profile_action_status_default" in source
