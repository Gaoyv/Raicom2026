#!/usr/bin/env python3
# 速度坐标系诊断：分别施加 前进/横移/旋转 指令，量出 odom 实际位移方向与速度，
# 用来确定 lateral_velocity 是否可用、正负号指向哪边，为"终点纯平移精修"提供依据。
#
# 运行：先在仿真里 Reset 让机器人站好，再：
#   ./run.sh usercode/py/diag_velocity.py --no-reset-prompt

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


class DiagVelocity(Node):
    def __init__(self, odom_topic):
        super().__init__("diag_velocity")
        self.publisher = self.create_publisher(McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10)
        self.client = self.create_client(SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
        self.action_client = self.create_client(SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction")
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.on_odom, odom_qos)
        self.source = "node"
        self.odom_topic = odom_topic
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

    def run_segment(self, label, forward, lateral, angular, duration):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        x0, y0, yaw0 = self.x, self.y, self.yaw
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(forward=forward, lateral=lateral, angular=angular)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)
        self.stop()
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        dx, dy = self.x - x0, self.y - y0
        dyaw = math.degrees(self.yaw - yaw0)
        # 机体系分解：把世界位移投影到 起始朝向的前向/左向
        fwd_hat = (math.cos(yaw0), math.sin(yaw0))
        left_hat = (-math.sin(yaw0), math.cos(yaw0))
        body_fwd = dx * fwd_hat[0] + dy * fwd_hat[1]
        body_left = dx * left_hat[0] + dy * left_hat[1]
        self.get_logger().info(
            f"[{label}] cmd(fwd={forward},lat={lateral},ang={angular}) {duration}s | "
            f"world dx={dx:+.3f} dy={dy:+.3f} dyaw={dyaw:+.1f} | "
            f"body forward={body_fwd:+.3f} left={body_left:+.3f} | "
            f"start yaw={math.degrees(yaw0):.1f}"
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
    parser.add_argument("--speed", type=float, default=0.2)
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = DiagVelocity(args.odom_topic)
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
        node.run_segment("FORWARD+", args.speed, 0.0, 0.0, args.seconds)
        node.run_segment("LATERAL+", 0.0, args.speed, 0.0, args.seconds)
        node.run_segment("LATERAL-", 0.0, -args.speed, 0.0, args.seconds)
    finally:
        node.stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
