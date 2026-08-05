#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FakeNav2(Node):
    def __init__(self):
        super().__init__("rs_control_fake_nav2")
        self._proposal = self.create_publisher(
            Twist, "/rs_control/navigation/controller_proposal", 1
        )
        self._server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self._execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

    def _execute(self, goal_handle):
        feedback = NavigateToPose.Feedback()
        feedback.current_pose = goal_handle.request.pose
        feedback.estimated_time_remaining.sec = 1
        for step in range(20):
            if goal_handle.is_cancel_requested:
                self._proposal.publish(Twist())
                goal_handle.canceled()
                return NavigateToPose.Result()
            proposal = Twist()
            proposal.linear.x = 0.2
            proposal.angular.z = 0.1
            self._proposal.publish(proposal)
            feedback.navigation_time.nanosec = step * 50_000_000
            feedback.distance_remaining = float(20 - step) / 20.0
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)
        self._proposal.publish(Twist())
        goal_handle.succeed()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeNav2()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
