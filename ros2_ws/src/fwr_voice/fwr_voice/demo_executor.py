"""Simulation-only high-level task executor."""

import time

from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist

from fwr_interfaces.action import ExecuteRobotTask

from .contracts import TASK_TYPES
from .jog import parse_motion_target, run_motion


class DemoExecutor(Node):
    def __init__(self):
        super().__init__("voice_demo_executor")
        self.server = ActionServer(self, ExecuteRobotTask, "/robot/task/execute",
                                   execute_callback=self.execute,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())
        self.demo_velocity = self.create_publisher(Twist, "/demo/cmd_vel", 10)

    def _publish_demo_velocity(self, linear, angular):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.demo_velocity.publish(command)

    def execute(self, handle):
        goal = handle.request
        result = ExecuteRobotTask.Result()
        if goal.task_type not in TASK_TYPES:
            handle.abort()
            result.status, result.message = "rejected", "不支持该任务"
            return result
        if goal.task_type == "SIM_JOG":
            try:
                parse_motion_target(goal.target_id)
            except ValueError:
                handle.abort()
                result.status, result.message = "rejected", "不支持该运动方向"
                return result
            canceled = run_motion(goal.target_id, self._publish_demo_velocity,
                                  lambda: handle.is_cancel_requested, time.sleep)
            if canceled:
                handle.canceled()
                result.status, result.message = "canceled", "模拟底盘已停止"
                return result
            result.message = "模拟底盘动作已完成"
        elif goal.task_type == "GO_TO_NAMED_LOCATION":
            for _ in range(20):
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.status, result.message = "canceled", "模拟任务已取消"
                    return result
                feedback = ExecuteRobotTask.Feedback()
                feedback.status, feedback.message = "running", "模拟移动中"
                handle.publish_feedback(feedback)
                time.sleep(0.1)
            result.message = f"模拟到达 {goal.target_id}"
        elif goal.task_type == "QUERY_STATUS":
            result.message = "模拟执行器空闲"
        else:
            result.message = "已请求停止"
        handle.succeed()
        result.success, result.status = True, "completed"
        return result


def main():
    rclpy.init()
    node = DemoExecutor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
