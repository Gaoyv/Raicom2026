#!/usr/bin/env python3

import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader, RequestHeader, CommonState, McActionCommand
from aimdk_msgs.srv import SetMcInputSource, SetMcAction


class WalkForwardSteps(Node):
    def __init__(self, steps, step_duration):
        super().__init__("walk_forward_steps")
        self.publisher = self.create_publisher(
            McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10
        )
        self.client = self.create_client(
            SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource"
        )
        self.action_client = self.create_client(
            SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction"
        )

        self.steps = steps
        self.step_duration = step_duration
        self.forward_velocity = 0.5  # m/s
        self.timer = None

    def register_input_source(self):
        timeout_sec = 8.0
        start = self.get_clock().now().nanoseconds / 1e9

        while not self.client.wait_for_service(timeout_sec=2.0):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                self.get_logger().error("Waiting for input source service timed out")
                return False
            self.get_logger().info("Waiting for input source service...")

        req = SetMcInputSource.Request()
        req.action.value = 1001  # INPUTACTION_ADD
        req.input_source.name = "node"
        req.input_source.priority = 40
        req.input_source.timeout = 1000

        for i in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"trying to register input source... [{i}]")

        if future.done():
            try:
                response = future.result()
                state = response.response.state.value
                self.get_logger().info(
                    f"Input source set successfully: state={state}, task_id={response.response.task_id}"
                )
                return True
            except Exception as exc:
                self.get_logger().error(f"Service call exception: {exc}")
                return False

        self.get_logger().error("Service call failed or timed out")
        return False

    def set_locomotion_mode(self):
        timeout_sec = 8.0
        start = self.get_clock().now().nanoseconds / 1e9

        while not self.action_client.wait_for_service(timeout_sec=2.0):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                self.get_logger().error("Waiting for SetMcAction service timed out")
                return False
            self.get_logger().info("Waiting for SetMcAction service...")

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = "node"
        cmd = McActionCommand()
        cmd.action_desc = "LOCOMOTION_DEFAULT"
        req.command = cmd

        for i in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            future = self.action_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"Trying to set locomotion mode... [{i}]")

        if future.done():
            try:
                response = future.result()
                if response.response.status.value == CommonState.SUCCESS:
                    self.get_logger().info("Locomotion mode set successfully")
                    return True
                else:
                    self.get_logger().error(f"Failed to set locomotion mode: {response.response.message}")
                    return False
            except Exception as exc:
                self.get_logger().error(f"Service call exception: {exc}")
                return False

        self.get_logger().error("Service call failed or timed out")
        return False

    def start_publish(self):
        if not self.timer:
            self.timer = self.create_timer(0.02, self.publish_velocity)

    def publish_velocity(self):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = self.forward_velocity
        msg.lateral_velocity = 0.0
        msg.angular_velocity = 0.0
        self.publisher.publish(msg)

    def stop(self):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = 0.0
        msg.lateral_velocity = 0.0
        msg.angular_velocity = 0.0
        self.publisher.publish(msg)


_global_node = None


def _signal_handler(sig, _frame):
    if _global_node is not None:
        _global_node.stop()
        _global_node.get_logger().info(
            f"Received signal {sig}, stopping and shutting down"
        )
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


def _parse_args():
    parser = argparse.ArgumentParser(description="Walk forward specified number of steps")
    parser.add_argument("--steps", type=int, default=10, help="Number of steps to walk")
    parser.add_argument("--step-duration", type=float, default=0.8, help="Duration of each step in seconds")
    parser.add_argument("--velocity", type=float, default=0.5, help="Forward velocity in m/s")
    return parser.parse_args()


def main():
    global _global_node

    args = _parse_args()

    rclpy.init()
    node = WalkForwardSteps(args.steps, args.step_duration)
    _global_node = node

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Register input source
    if not node.register_input_source():
        node.get_logger().error("Input source registration failed, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    # Set locomotion mode
    if not node.set_locomotion_mode():
        node.get_logger().error("Failed to set locomotion mode, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    # Set forward velocity based on argument
    node.forward_velocity = args.velocity

    # Calculate total duration for the specified number of steps
    total_duration = args.steps * args.step_duration
    node.get_logger().info(
        f"Walking forward {args.steps} steps: velocity={args.velocity:.2f} m/s, step_duration={args.step_duration:.2f}s, total_duration={total_duration:.2f}s"
    )

    node.start_publish()
    start = node.get_clock().now()
    while (node.get_clock().now() - start).nanoseconds / 1e9 < total_duration:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.001)

    node.stop()
    node.get_logger().info(f"Finished walking {args.steps} steps, robot stopped")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
