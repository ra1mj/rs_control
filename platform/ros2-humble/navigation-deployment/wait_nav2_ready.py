#!/usr/bin/env python3
"""Wait for the aggregate Nav2 lifecycle state and action discovery."""

import argparse
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("action_name")
    parser.add_argument("timeout_seconds", type=int)
    parser.add_argument("lifecycle_manager")
    args = parser.parse_args()
    if not args.action_name.startswith("/"):
        parser.error("action name must be absolute")
    if not args.lifecycle_manager.startswith("/"):
        parser.error("lifecycle manager name must be absolute")
    if not 1 <= args.timeout_seconds <= 300:
        parser.error("timeout seconds must be in 1..300")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = Node("rs_control_navigation_readiness_probe")
    action_client = ActionClient(node, NavigateToPose, args.action_name)
    lifecycle_client = node.create_client(
        Trigger, f"{args.lifecycle_manager}/is_active"
    )
    deadline = time.monotonic() + args.timeout_seconds
    try:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            wait_slice = min(0.5, max(0.0, remaining))
            if not action_client.wait_for_server(timeout_sec=wait_slice):
                continue
            if not lifecycle_client.wait_for_service(timeout_sec=wait_slice):
                continue
            future = lifecycle_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=min(1.0, max(0.0, remaining))
            )
            if future.done() and future.result() is not None:
                response = future.result()
                if response.success:
                    return 0
            time.sleep(0.1)
        print(
            "Nav2 did not become ready: "
            f"action={args.action_name} manager={args.lifecycle_manager}",
            file=sys.stderr,
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
