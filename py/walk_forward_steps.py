#!/usr/bin/env python3

import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader, RequestHeader, CommonState, McActionCommand
from aimdk_msgs.srv import SetMcInputSource, SetMcAction
from nav_msgs.msg import Odometry


class WalkForwardSteps(Node):
    def __init__(self, steps, step_duration, odom_topic, pose_log_interval):
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

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, odom_qos
        )

        self.steps = steps
        self.step_duration = step_duration
        self.forward_velocity = 0.5  # m/s
        self.odom_topic = odom_topic
        self.pose_log_interval = pose_log_interval
        self.last_pose_log_time = 0.0

        self.odom_ready = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_ready = True

    def wait_for_odom(self, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_ready:
                self.log_pose(prefix=f"Odometry ready on {self.odom_topic}")
                return True
        self.get_logger().error(f"Timed out waiting for {self.odom_topic}")
        return False

    def log_pose(self, prefix="Current pose"):
        self.get_logger().info(
            f"{prefix}: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f} deg"
        )

    def maybe_log_pose(self):
        now = time.monotonic()
        if now - self.last_pose_log_time >= self.pose_log_interval:
            self.last_pose_log_time = now
            self.log_pose()

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

    def set_mode(self, action_desc, source="node"):
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
        req.source = source
        cmd = McActionCommand()
        cmd.action_desc = action_desc
        req.command = cmd

        for i in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            future = self.action_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"Trying to set {action_desc}... [{i}]")

        if future.done():
            try:
                response = future.result()
                if response.response.status.value == CommonState.SUCCESS:
                    self.get_logger().info(f"{action_desc} set successfully")
                    return True
                else:
                    self.get_logger().error(
                        f"Failed to set {action_desc}: {response.response.message}"
                    )
                    return False
            except Exception as exc:
                self.get_logger().error(f"Service call exception: {exc}")
                return False

        self.get_logger().error("Service call failed or timed out")
        return False

    def set_locomotion_mode(self):
        return self.set_mode("LOCOMOTION_DEFAULT")

    def publish_velocity(self):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "node"
        msg.forward_velocity = self.forward_velocity
        msg.lateral_velocity = 0.0
        msg.angular_velocity = 0.0
        self.publisher.publish(msg)

    def walk_for_steps(self, steps):
        total_duration = steps * self.step_duration
        self.last_pose_log_time = 0.0
        self.get_logger().info(
            f"Walking estimated {steps:.2f} steps: velocity={self.forward_velocity:.2f} m/s, "
            f"step_duration={self.step_duration:.2f}s, total_duration={total_duration:.2f}s"
        )

        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < total_duration:
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.02)
            self.maybe_log_pose()
            time.sleep(0.02)

        self.stop()
        self.log_pose(prefix="Final pose")
        self.get_logger().info("Finished walking estimated steps, robot stopped")

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
    parser = argparse.ArgumentParser(description="Walk forward specified number of estimated steps")
    parser.add_argument(
        "--no-reset-prompt",
        action="store_true",
        help="Do not wait for manual simulator Reset before standing up",
    )
    parser.add_argument(
        "--stand-time",
        type=float,
        default=1.5,
        help="Seconds to wait after Reset before switching to locomotion",
    )
    parser.add_argument("--steps", type=float, default=10.0, help="Estimated step count to walk in non-interactive mode")
    parser.add_argument("--step-duration", type=float, default=0.8, help="Duration of each estimated step in seconds")
    parser.add_argument("--velocity", type=float, default=0.5, help="Forward velocity in m/s")
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Prompt repeatedly for step counts; input q or blank line to quit",
    )
    parser.add_argument(
        "--odom-topic",
        type=str,
        default="/aima/hal/odom/state",
        help="Odometry topic used for real-time pose printing",
    )
    parser.add_argument(
        "--pose-log-interval",
        type=float,
        default=0.5,
        help="Seconds between pose logs while walking",
    )
    return parser.parse_args()


def _interactive_loop(node):
    print(
        "\nInteractive mode is active. Enter an estimated step count to walk forward.\n"
        "Examples: 1, 2, 3.5\n"
        "Enter q or press Enter on an empty line to quit.\n",
        flush=True,
    )

    while rclpy.ok():
        try:
            raw = input("Estimated steps> ").strip()
        except EOFError:
            print()
            break

        if raw == "" or raw.lower() == "q":
            break

        try:
            steps = float(raw)
        except ValueError:
            print("Invalid input. Please enter a number, q, or blank line.", flush=True)
            continue

        if steps <= 0.0:
            print("Please enter a positive number of estimated steps.", flush=True)
            continue

        node.walk_for_steps(steps)


def main():
    global _global_node

    args = _parse_args()

    rclpy.init()
    node = WalkForwardSteps(
        args.steps,
        args.step_duration,
        args.odom_topic,
        args.pose_log_interval,
    )
    _global_node = node

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Set stand mode first, matching simulator startup flow in reference examples.
    if not node.set_mode("STAND_DEFAULT"):
        node.get_logger().error("Failed to set stand mode, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    if not args.no_reset_prompt:
        print(
            "\nSTAND_DEFAULT is active. Click Reset in the simulator now; "
            "the robot should appear standing in the start area.\n"
            "Press Enter here after Reset to continue...",
            flush=True,
        )
        input()

    time.sleep(args.stand_time)

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

    if not node.wait_for_odom():
        if rclpy.ok():
            rclpy.shutdown()
        return

    # Set forward velocity based on argument
    node.forward_velocity = args.velocity

    try:
        if args.interactive:
            _interactive_loop(node)
        else:
            node.walk_for_steps(args.steps)
    finally:
        node.stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
