#!/usr/bin/env python3

import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from aimdk_msgs.msg import (
    CommonState,
    McActionCommand,
    McLocomotionVelocity,
    MessageHeader,
    RequestHeader,
)
from aimdk_msgs.srv import SetMcAction, SetMcInputSource
from nav_msgs.msg import Odometry


class ClosedLoopPathController(Node):
    def __init__(self, odom_topic, forward_speed, turn_speed, distance_kp, heading_kp):
        super().__init__("closed_loop_path_controller")
        self.publisher = self.create_publisher(
            McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10
        )
        self.mode_client = self.create_client(
            SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction"
        )
        self.input_client = self.create_client(
            SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource"
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
        self.seq = 0
        self.odom_topic = odom_topic
        self.forward_speed = float(forward_speed)
        self.turn_speed = float(turn_speed)
        self.distance_kp = float(distance_kp)
        self.heading_kp = float(heading_kp)

        self.max_heading_correction = 0.35
        self.max_distance_speed = self.forward_speed
        self.min_distance_speed = 0.10
        self.max_turn_speed = self.turn_speed
        self.min_turn_speed = 0.10
        self.position_tolerance = 0.05
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
                self.get_logger().info(
                    f"Odometry ready on {self.odom_topic}: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f} deg"
                )
                return True
        self.get_logger().error(f"Timed out waiting for {self.odom_topic}")
        return False

    def register_input_source(self):
        timeout_sec = 8.0
        start = self.get_clock().now().nanoseconds / 1e9

        while not self.input_client.wait_for_service(timeout_sec=2.0):
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
            future = self.input_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"trying to register input source... [{i}]")

        if not future.done() or future.result() is None:
            self.get_logger().error("Input source registration failed")
            return False

        response = future.result()
        self.get_logger().info(
            f"Input source registered: state={response.response.state.value}, "
            f"task_id={response.response.task_id}"
        )
        return True

    def set_locomotion_mode(self):
        timeout_sec = 8.0
        start = self.get_clock().now().nanoseconds / 1e9

        while not self.mode_client.wait_for_service(timeout_sec=2.0):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                self.get_logger().error("Waiting for SetMcAction service timed out")
                return False
            self.get_logger().info("Waiting for SetMcAction service...")

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = self.source
        cmd = McActionCommand()
        cmd.action_desc = "LOCOMOTION_DEFAULT"
        req.command = cmd

        for i in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            future = self.mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"trying to set locomotion mode... [{i}]")

        if not future.done() or future.result() is None:
            self.get_logger().error("SetMcAction call failed or timed out")
            return False

        response = future.result()
        if response.response.status.value == CommonState.SUCCESS:
            self.get_logger().info("LOCOMOTION_DEFAULT set successfully")
            return True

        self.get_logger().error(
            f"Failed to set locomotion mode: {response.response.message}"
        )
        return False

    def publish_velocity(self, forward=0.0, lateral=0.0, angular=0.0):
        msg = McLocomotionVelocity()
        msg.header = self.make_header()
        msg.source = self.source
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self.publisher.publish(msg)

    def stop(self, repeats=10):
        for _ in range(repeats):
            self.publish_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def make_header(self):
        header = MessageHeader()
        now = self.get_clock().now().to_msg()
        header.stamp = now
        header.meas_stamp = now
        header.frame_id = "base_link"
        header.sequence = self.seq
        self.seq += 1
        return header

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def drive_straight(self, distance):
        start_x = self.x
        start_y = self.y
        target_heading = self.yaw
        direction = 1.0 if distance >= 0.0 else -1.0
        target_distance = abs(distance)

        self.get_logger().info(
            f"Drive straight {distance:.2f} m from x={start_x:.3f}, y={start_y:.3f}, "
            f"heading={math.degrees(target_heading):.1f} deg"
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

            traveled = math.hypot(self.x - start_x, self.y - start_y)
            remaining = target_distance - traveled
            if remaining <= self.position_tolerance:
                break

            heading_error = self.normalize_angle(target_heading - self.yaw)
            angular = max(
                -self.max_heading_correction,
                min(self.max_heading_correction, self.heading_kp * heading_error),
            )
            forward = max(
                self.min_distance_speed,
                min(self.max_distance_speed, self.distance_kp * remaining),
            ) * direction

            self.publish_velocity(forward=forward, angular=angular)
            time.sleep(0.02)

        self.stop()
        self.get_logger().info("Straight segment finished")

    def turn_in_place(self, angle):
        start_yaw = self.yaw
        target_yaw = self.normalize_angle(start_yaw + angle)
        direction = 1.0 if angle >= 0.0 else -1.0

        self.get_logger().info(
            f"Turn {math.degrees(angle):.1f} deg from {math.degrees(start_yaw):.1f} deg "
            f"to {math.degrees(target_yaw):.1f} deg"
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            error = self.normalize_angle(target_yaw - self.yaw)
            if abs(error) <= self.angle_tolerance:
                break

            angular = max(
                self.min_turn_speed,
                min(self.max_turn_speed, abs(error) * self.heading_kp),
            ) * direction
            self.publish_velocity(angular=angular)
            time.sleep(0.02)

        self.stop()
        self.get_logger().info("Turn segment finished")


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
        description="Closed-loop path: forward 10 m, clockwise 90 deg, forward 5 m"
    )
    parser.add_argument(
        "--odom-topic",
        type=str,
        default="/aima/hal/odom/state",
        help="Odometry topic used for closed-loop feedback",
    )
    parser.add_argument(
        "--forward-1", type=float, default=10.0, help="First straight distance in meters"
    )
    parser.add_argument(
        "--turn-deg", type=float, default=-90.0, help="Turn angle in degrees, clockwise is negative"
    )
    parser.add_argument(
        "--forward-2", type=float, default=5.0, help="Second straight distance in meters"
    )
    parser.add_argument(
        "--forward-speed", type=float, default=0.4, help="Maximum forward speed in m/s"
    )
    parser.add_argument(
        "--turn-speed", type=float, default=0.5, help="Maximum turn speed in rad/s"
    )
    parser.add_argument(
        "--distance-kp", type=float, default=0.6, help="Proportional gain for distance control"
    )
    parser.add_argument(
        "--heading-kp", type=float, default=1.2, help="Proportional gain for heading control"
    )
    return parser.parse_args()


def main():
    global _global_node

    args = _parse_args()
    rclpy.init()
    node = ClosedLoopPathController(
        args.odom_topic,
        args.forward_speed,
        args.turn_speed,
        args.distance_kp,
        args.heading_kp,
    )
    _global_node = node

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        if not node.wait_for_odom():
            return
        if not node.register_input_source():
            return
        if not node.set_locomotion_mode():
            return

        time.sleep(1.0)
        node.drive_straight(args.forward_1)
        time.sleep(0.5)
        node.turn_in_place(math.radians(args.turn_deg))
        time.sleep(0.5)
        node.drive_straight(args.forward_2)
        node.get_logger().info("Closed-loop path finished")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
