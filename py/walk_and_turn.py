#!/usr/bin/env python3

import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader, RequestHeader, CommonState, McActionCommand
from aimdk_msgs.srv import SetMcInputSource, SetMcAction


class WalkAndTurn(Node):
    def __init__(self):
        super().__init__("walk_and_turn")
        self.publisher = self.create_publisher(
            McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10
        )
        self.client = self.create_client(
            SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource"
        )
        self.action_client = self.create_client(
            SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction"
        )

        self.timer = None
        self.forward_velocity = 0.0
        self.angular_velocity = 0.0

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
        msg.angular_velocity = self.angular_velocity
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


def main():
    global _global_node

    rclpy.init()
    node = WalkAndTurn()
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

    node.start_publish()

    # Step 1: Walk forward 9 steps (9 * 0.8s = 7.2s)
    forward_velocity = 0.5  # m/s
    step_duration = 0.8  # seconds per step
    num_steps_1 = 9
    duration_1 = num_steps_1 * step_duration

    node.get_logger().info(f"Step 1: Walking forward {num_steps_1} steps for {duration_1:.2f}s")
    node.forward_velocity = forward_velocity
    node.angular_velocity = 0.0

    start = node.get_clock().now()
    while (node.get_clock().now() - start).nanoseconds / 1e9 < duration_1:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.001)

    # Step 2: Turn clockwise 90 degrees (π/2 radians)
    # Clockwise is negative angular velocity
    angular_velocity = -0.8  # rad/s (clockwise)
    angle_to_turn = math.pi / 2  # 90 degrees in radians
    duration_2 = angle_to_turn / abs(angular_velocity)

    node.get_logger().info(f"Step 2: Turning clockwise 90 degrees for {duration_2:.2f}s")
    node.forward_velocity = 0.0
    node.angular_velocity = angular_velocity

    start = node.get_clock().now()
    while (node.get_clock().now() - start).nanoseconds / 1e9 < duration_2:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.001)

    # Step 3: Walk forward 3 steps (3 * 0.8s = 2.4s)
    num_steps_2 = 3
    duration_3 = num_steps_2 * step_duration

    node.get_logger().info(f"Step 3: Walking forward {num_steps_2} steps for {duration_3:.2f}s")
    node.forward_velocity = forward_velocity
    node.angular_velocity = 0.0

    start = node.get_clock().now()
    while (node.get_clock().now() - start).nanoseconds / 1e9 < duration_3:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.001)

    node.stop()
    node.get_logger().info("All movements completed, robot stopped")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
