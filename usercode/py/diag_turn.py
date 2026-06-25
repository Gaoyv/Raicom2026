#!/usr/bin/env python3
# 原地转身漂移诊断：测不同角速度下"原地转身"造成的位置平移有多大、是否可重复，
# 决定能否靠"慢转减小漂移"或"固定量预补偿"来保住 x 精度。
#
# 运行：先在仿真里 Reset 站好，再：
#   ./run.sh usercode/py/diag_turn.py --no-reset-prompt

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


class DiagTurn(Node):
    def __init__(self, odom_topic):
        super().__init__("diag_turn")
        self.publisher = self.create_publisher(McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10)
        self.client = self.create_client(SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
        self.action_client = self.create_client(SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction")
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(Odometry, odom_topic, self.on_odom, odom_qos)
        self.source = "node"
        self.odom_ready = False
        self.x = self.y = self.yaw = 0.0

    def on_odom(self, msg):
        p = msg.pose.pose
        self.x = p.position.x
        self.y = p.position.y
        q = p.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_ready = True

    def wait_for_odom(self, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom_ready:
                return True
        return False

    def register_input_source(self):
        if not self.client.wait_for_service(timeout_sec=8.0):
            return False
        req = SetMcInputSource.Request()
        req.action.value = 1001
        req.input_source.name = self.source
        req.input_source.priority = 40
        req.input_source.timeout = 1000
        for _ in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                return True
        return False

    def set_mode(self, action_desc):
        if not self.action_client.wait_for_service(timeout_sec=8.0):
            return False
        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = self.source
        cmd = McActionCommand()
        cmd.action_desc = action_desc
        req.command = cmd
        for _ in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            future = self.action_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                return True
        return False

    def publish_velocity(self, forward=0.0, lateral=0.0, angular=0.0):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = self.source
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self.publisher.publish(msg)

    def stop(self, repeats=10):
        for _ in range(repeats):
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    @staticmethod
    def norm(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def turn_by(self, label, delta_deg, turn_speed):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        x0, y0, yaw0 = self.x, self.y, self.yaw
        target = yaw0 + math.radians(delta_deg)
        deadline = time.monotonic() + 20.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            err = self.norm(target - self.yaw)
            if abs(err) <= math.radians(2.0) or time.monotonic() >= deadline:
                break
            ang = max(0.1, min(turn_speed, abs(err) * 1.2))
            self.publish_velocity(angular=ang if err > 0 else -ang)
            time.sleep(0.02)
        self.stop()
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        dx, dy = self.x - x0, self.y - y0
        self.get_logger().info(
            f"[{label}] turn {delta_deg:+.0f}deg @ spd={turn_speed} | "
            f"pos drift dx={dx:+.3f} dy={dy:+.3f} (|d|={math.hypot(dx, dy):.3f}) | "
            f"yaw {math.degrees(yaw0):.1f}->{math.degrees(self.yaw):.1f}"
        )


_node = None


def _sig(sig, _f):
    if _node is not None:
        _node.stop()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


def main():
    global _node
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-reset-prompt", action="store_true")
    parser.add_argument("--odom-topic", default="/aima/hal/odom/state")
    args = parser.parse_args()

    rclpy.init()
    node = DiagTurn(args.odom_topic)
    _node = node
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    if not node.set_mode("STAND_DEFAULT"):
        node.get_logger().error("stand failed"); rclpy.shutdown(); return
    time.sleep(1.5)
    if not node.register_input_source():
        node.get_logger().error("register failed"); rclpy.shutdown(); return
    if not node.set_mode("LOCOMOTION_DEFAULT"):
        node.get_logger().error("locomotion failed"); rclpy.shutdown(); return
    if not node.wait_for_odom():
        node.get_logger().error("no odom"); rclpy.shutdown(); return

    try:
        # 慢转两次 90 (累积 180)，再快转 -90 两次，对比漂移大小与重复性
        node.turn_by("SLOW-A", 90.0, 0.25)
        node.turn_by("SLOW-B", 90.0, 0.25)
        node.turn_by("FAST-A", -90.0, 0.6)
        node.turn_by("FAST-B", -90.0, 0.6)
    finally:
        node.stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
