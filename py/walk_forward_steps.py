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
    def __init__(
        self,
        odom_topic,
        target_x,
        target_y,
        target_yaw_deg,
        forward_speed,
        turn_speed,
        distance_kp,
        heading_kp,
        pose_log_interval,
    ):
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

        self.source = "node"
        self.odom_topic = odom_topic
        self.target_x = float(target_x)
        self.target_y = float(target_y)
        self.target_yaw = math.radians(float(target_yaw_deg))
        self.forward_speed = float(forward_speed)
        self.turn_speed = float(turn_speed)
        self.distance_kp = float(distance_kp)
        self.heading_kp = float(heading_kp)
        self.pose_log_interval = float(pose_log_interval)
        self.last_pose_log_time = 0.0

        self.max_heading_correction = 0.35
        self.max_distance_speed = self.forward_speed
        self.min_distance_speed = 0.05
        self.max_turn_speed = self.turn_speed
        self.min_turn_speed = 0.10
        self.position_tolerance = 0.03
        self.angle_tolerance = math.radians(2.0)

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
        req.action.value = 1001
        req.input_source.name = self.source
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

    def publish_velocity(self, forward=0.0, lateral=0.0, angular=0.0):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = self.source
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self.publisher.publish(msg)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def turn_to_heading(self, target_heading):
        self.last_pose_log_time = 0.0
        self.get_logger().info(
            f"Turning to heading {math.degrees(target_heading):.1f} deg"
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            error = self.normalize_angle(target_heading - self.yaw)
            if abs(error) <= self.angle_tolerance:
                break

            angular = max(
                self.min_turn_speed,
                min(self.max_turn_speed, abs(error) * self.heading_kp),
            )
            if error < 0.0:
                angular = -angular

            self.publish_velocity(angular=angular)
            self.maybe_log_pose()
            time.sleep(0.02)

        self.stop()
        self.log_pose(prefix="Heading aligned")

    def drive_axis_to_target(self, axis_name, target_value, target_heading):
        self.last_pose_log_time = 0.0
        start_error = (
            target_value - self.y if axis_name == "y" else target_value - self.x
        )
        previous_error = start_error

        self.get_logger().info(
            f"Driving axis {axis_name} toward {target_value:.3f} with heading {math.degrees(target_heading):.1f} deg"
        )

        stop_reason = "tolerance"
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

            current_value = self.y if axis_name == "y" else self.x
            error = target_value - current_value

            if abs(error) <= self.position_tolerance:
                stop_reason = "tolerance"
                break

            if start_error != 0.0 and previous_error != 0.0 and error * previous_error < 0.0:
                stop_reason = "crossing"
                break

            heading_error = self.normalize_angle(target_heading - self.yaw)
            angular = max(
                -self.max_heading_correction,
                min(self.max_heading_correction, self.heading_kp * heading_error),
            )
            forward = max(
                self.min_distance_speed,
                min(self.max_distance_speed, self.distance_kp * abs(error)),
            )

            self.publish_velocity(forward=forward, angular=angular)
            self.maybe_log_pose()
            previous_error = error
            time.sleep(0.02)

        self.stop()
        final_value = self.y if axis_name == "y" else self.x
        final_error = target_value - final_value
        self.get_logger().info(
            f"Segment finished on axis {axis_name}: value={final_value:.3f}, "
            f"target={target_value:.3f}, error={final_error:.3f}, stop_reason={stop_reason}"
        )

    def move_to_target(self):
        self.log_pose(prefix="Start pose")

        dx = self.target_x - self.x
        dy = self.target_y - self.y

        self.get_logger().info(
            f"Target point: x={self.target_x:.3f}, y={self.target_y:.3f}, dx={dx:.3f}, dy={dy:.3f}"
        )

        if abs(dy) > self.position_tolerance:
            y_heading = math.pi / 2.0 if dy >= 0.0 else -math.pi / 2.0
            self.get_logger().info(
                f"Stage 1: correct y by {dy:.3f} m with heading {math.degrees(y_heading):.1f} deg"
            )
            self.turn_to_heading(y_heading)
            self.drive_axis_to_target("y", self.target_y, y_heading)

        dx_after = self.target_x - self.x
        dy_after = self.target_y - self.y
        self.get_logger().info(
            f"After stage 1: dx={dx_after:.3f}, dy={dy_after:.3f}"
        )

        if abs(dx_after) > self.position_tolerance:
            x_heading = 0.0 if dx_after >= 0.0 else math.pi
            self.get_logger().info(
                f"Stage 2: correct x by {dx_after:.3f} m with heading {math.degrees(x_heading):.1f} deg"
            )
            self.turn_to_heading(x_heading)
            self.drive_axis_to_target("x", self.target_x, x_heading)

        self.get_logger().info(
            f"Stage 3: turn to final heading {math.degrees(self.target_yaw):.1f} deg"
        )
        self.turn_to_heading(self.target_yaw)

        final_dx = self.target_x - self.x
        final_dy = self.target_y - self.y
        self.stop()
        self.get_logger().info(
            f"Navigation finished: final x={self.x:.3f}, y={self.y:.3f}, "
            f"error dx={final_dx:.3f}, dy={final_dy:.3f}"
        )

    def stop(self, repeats=10):
        for _ in range(repeats):
            self.publish_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)


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
    parser = argparse.ArgumentParser(
        description="Closed-loop navigation to a target point using odometry"
    )
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
    parser.add_argument(
        "--odom-topic",
        type=str,
        default="/aima/hal/odom/state",
        help="Odometry topic used for closed-loop feedback",
    )
    parser.add_argument(
        "--target-x",
        type=float,
        default=-0.020,
        help="Target x position in map frame",
    )
    parser.add_argument(
        "--target-y",
        type=float,
        default=1.745,
        help="Target y position in map frame",
    )
    parser.add_argument(
        "--target-yaw-deg",
        type=float,
        default=-90.0,
        help="Final target yaw in degrees after reaching the target point",
    )
    parser.add_argument(
        "--forward-speed",
        type=float,
        default=0.25,
        help="Maximum forward speed in m/s",
    )
    parser.add_argument(
        "--turn-speed",
        type=float,
        default=0.35,
        help="Maximum turn speed in rad/s",
    )
    parser.add_argument(
        "--distance-kp",
        type=float,
        default=0.6,
        help="Proportional gain for distance control",
    )
    parser.add_argument(
        "--heading-kp",
        type=float,
        default=1.2,
        help="Proportional gain for heading control",
    )
    parser.add_argument(
        "--pose-log-interval",
        type=float,
        default=0.5,
        help="Seconds between pose logs while moving",
    )
    return parser.parse_args()


def main():
    global _global_node

    args = _parse_args()

    rclpy.init()
    node = WalkForwardSteps(
        args.odom_topic,
        args.target_x,
        args.target_y,
        args.target_yaw_deg,
        args.forward_speed,
        args.turn_speed,
        args.distance_kp,
        args.heading_kp,
        args.pose_log_interval,
    )
    _global_node = node

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

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

    if not node.register_input_source():
        node.get_logger().error("Input source registration failed, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    if not node.set_locomotion_mode():
        node.get_logger().error("Failed to set locomotion mode, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    if not node.wait_for_odom():
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        node.move_to_target()
    finally:
        node.stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
