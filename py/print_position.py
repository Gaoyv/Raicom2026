#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry


class PositionPrinter(Node):
    def __init__(self, odom_topic, interval):
        super().__init__("position_printer")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, qos
        )
        self.odom_topic = odom_topic
        self.interval = float(interval)

        self.odom_ready = False
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0
        self.last_print = 0.0

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def on_odom(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z
        q = pose.orientation
        self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.vx = twist.linear.x
        self.vy = twist.linear.y
        self.vz = twist.linear.z
        self.wz = twist.angular.z
        self.odom_ready = True

    def wait_for_odom(self, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_ready:
                self.get_logger().info(f"Subscribed to {self.odom_topic}")
                return True
        self.get_logger().error(f"Timed out waiting for {self.odom_topic}")
        return False

    def maybe_print(self):
        now = time.monotonic()
        if now - self.last_print >= self.interval:
            self.last_print = now
            self.get_logger().info(
                f"Position: x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
                f"yaw={math.degrees(self.yaw):.1f} deg, "
                f"linear=[{self.vx:.3f}, {self.vy:.3f}, {self.vz:.3f}], "
                f"angular_z={self.wz:.3f}"
            )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Print robot position information at a fixed interval"
    )
    parser.add_argument(
        "--odom-topic",
        type=str,
        default="/aima/hal/odom/state",
        help="Odometry topic used to read robot position",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds between position prints",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = PositionPrinter(args.odom_topic, args.interval)

    try:
        if not node.wait_for_odom():
            return

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.maybe_print()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
