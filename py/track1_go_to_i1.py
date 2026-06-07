#!/usr/bin/env python3

import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import (
    CommonState,
    McActionCommand,
    McLocomotionVelocity,
    MessageHeader,
    PncTaskRequest,
    RequestHeader,
)
from aimdk_msgs.srv import SetMcAction, SetMcInputSource
from geometry_msgs.msg import PoseStamped


TASK_REQUEST_START = 1
TASK_REQUEST_STOP = 2
TASK_TYPE_PLANNING_NAVI_TO_POSE_2D = 2
PNC_MODE_NORMAL = 0


class Track1Controller(Node):
    def __init__(self):
        super().__init__("track1_go_to_i1")
        self.mode_client = self.create_client(
            SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction")
        self.input_client = self.create_client(
            SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
        self.vel_pub = self.create_publisher(
            McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10)
        self.pnc_pub = self.create_publisher(
            PncTaskRequest, "/aima/te/pnc_task_request", 10)
        self.seq = 0
        self.source = "node"
        self.task_id = int(time.time() * 1000)

    def set_mode(self, action_desc):
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetMcAction service is not available")
            return False

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = "rc"
        req.command = McActionCommand()
        req.command.action_desc = action_desc

        for i in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            future = self.mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"Retrying mode switch... [{i}]")

        if not future.done() or future.result() is None:
            self.get_logger().error(f"Set mode {action_desc} failed")
            return False

        response = future.result()
        ok = response.response.status.value == CommonState.SUCCESS
        if ok:
            self.get_logger().info(f"Mode switched to {action_desc}")
        else:
            self.get_logger().error(
                f"Set mode {action_desc} failed: {response.response.message}")
        return ok

    def register_input_source(self):
        if not self.input_client.wait_for_service(timeout_sec=8.0):
            self.get_logger().error("SetMcInputSource service is not available")
            return False

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
            self.get_logger().info(f"Retrying input source registration... [{i}]")

        if not future.done() or future.result() is None:
            self.get_logger().error("Input source registration failed")
            return False

        response = future.result()
        self.get_logger().info(
            f"Input source registered: state={response.response.state.value}, "
            f"task_id={response.response.task_id}")
        return True

    def publish_velocity(self, forward=0.0, lateral=0.0, angular=0.0):
        msg = McLocomotionVelocity()
        msg.header = self.make_header("base_link")
        msg.source = self.source
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self.vel_pub.publish(msg)

    def hold_velocity(self, duration, forward=0.0, lateral=0.0, angular=0.0):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(forward, lateral, angular)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)
        self.stop()

    def make_pnc_request(
            self, task_request, map_id, x=0.0, y=0.0, yaw=0.0,
            radius=0.1, pnc_mode=PNC_MODE_NORMAL, max_speed=0.25):
        msg = PncTaskRequest()
        msg.header = self.make_header("map")
        msg.task_type = TASK_TYPE_PLANNING_NAVI_TO_POSE_2D
        msg.task_request = task_request
        msg.pnc_mode = pnc_mode
        msg.task_id = self.task_id
        msg.map_id = map_id
        msg.target_pose_radius = radius
        msg.max_forward_speed = max_speed
        msg.reserve_info = [0] * 64

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = 0.0
        target.pose.orientation.z = math.sin(yaw / 2.0)
        target.pose.orientation.w = math.cos(yaw / 2.0)
        msg.target_pose = target
        return msg

    def publish_pnc_goal(
            self, map_id, x, y, yaw, radius, pnc_mode, max_speed, timeout):
        msg = self.make_pnc_request(
            TASK_REQUEST_START, map_id, x, y, yaw, radius, pnc_mode, max_speed)
        deadline = time.monotonic() + timeout

        self.get_logger().info(
            "PNC goal: x=%.2f y=%.2f yaw=%.2f radius=%.2f map_id=%d",
            x, y, yaw, radius, map_id)
        while rclpy.ok() and time.monotonic() < deadline:
            msg.header = self.make_header("map")
            msg.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.pnc_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.9)

    def stop_pnc(self, map_id):
        msg = self.make_pnc_request(TASK_REQUEST_STOP, map_id)
        for _ in range(3):
            self.pnc_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

    def stop(self, repeats=10):
        for _ in range(repeats):
            self.publish_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def make_header(self, frame_id):
        header = MessageHeader()
        now = self.get_clock().now().to_msg()
        header.stamp = now
        header.meas_stamp = now
        header.frame_id = frame_id
        header.sequence = self.seq
        self.seq += 1
        return header


node_for_signal = None


def signal_handler(sig, _frame):
    if node_for_signal is not None:
        node_for_signal.get_logger().info(
            f"Received signal {sig}, stopping robot")
        node_for_signal.stop()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Track 1: stand, move to I1 center, face the exit/I2.")
    parser.add_argument(
        "--no-reset-prompt",
        action="store_true",
        help="Do not wait for manual simulator Reset before moving.")
    parser.add_argument(
        "--stand-time",
        type=float,
        default=1.5,
        help="Seconds to wait after Reset before switching to locomotion.")
    parser.add_argument(
        "--control-mode",
        choices=("pnc", "velocity"),
        default="pnc",
        help="Use PNC goal navigation or timed velocity fallback.")
    parser.add_argument("--target-x", type=float, default=0.0)
    parser.add_argument("--target-y", type=float, default=1.7)
    parser.add_argument(
        "--target-yaw",
        type=float,
        default=-math.pi / 2.0,
        help="Final yaw at I1. -pi/2 faces I2 and the exit side.")
    parser.add_argument("--target-radius", type=float, default=0.10)
    parser.add_argument("--map-id", type=int, default=1773113429735)
    parser.add_argument("--pnc-mode", type=int, default=PNC_MODE_NORMAL)
    parser.add_argument("--pnc-speed", type=float, default=0.25)
    parser.add_argument(
        "--pnc-timeout",
        type=float,
        default=35.0,
        help="Seconds to keep publishing the PNC target.")
    parser.add_argument("--right-speed", type=float, default=0.2)
    parser.add_argument("--right-distance", type=float, default=1.5)
    parser.add_argument("--forward-speed", type=float, default=0.25)
    parser.add_argument("--forward-distance", type=float, default=3.2)
    parser.add_argument("--turn-speed", type=float, default=0.3)
    parser.add_argument("--turn-angle", type=float, default=math.pi)
    return parser.parse_args()


def main():
    global node_for_signal
    args = parse_args()
    rclpy.init()
    node = Track1Controller()
    node_for_signal = node
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        if not node.set_mode("STAND_DEFAULT"):
            return

        if not args.no_reset_prompt:
            print(
                "\nSTAND_DEFAULT is active. Click Reset in the simulator now; "
                "the robot should appear standing in the start area.\n"
                "Press Enter here after Reset to start moving...",
                flush=True)
            input()

        time.sleep(args.stand_time)

        if not node.set_mode("LOCOMOTION_DEFAULT"):
            return
        time.sleep(1.0)

        use_velocity_fallback = args.control_mode == "velocity"
        if args.control_mode == "pnc":
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.pnc_pub.get_subscription_count() > 0:
                    break

            if node.pnc_pub.get_subscription_count() > 0:
                node.publish_pnc_goal(
                    args.map_id,
                    args.target_x,
                    args.target_y,
                    args.target_yaw,
                    args.target_radius,
                    args.pnc_mode,
                    args.pnc_speed,
                    args.pnc_timeout,
                )
                node.stop_pnc(args.map_id)
                node.set_mode("STAND_DEFAULT")
                node.get_logger().info(
                    "PNC movement finished at I1 target pose")
                return

            node.get_logger().warning(
                "No subscriber on /aima/te/pnc_task_request; "
                "falling back to timed velocity control.")
            use_velocity_fallback = True

        if not use_velocity_fallback:
            return

        if not node.register_input_source():
            return

        # Initial pose in x2.xml: start (-1.5, -1.5), yaw ~= +90 deg.
        # I1 is (0, 1.7), I2/the exit side is at negative Y from I1.
        node.get_logger().info("Moving right toward x=0")
        node.hold_velocity(
            args.right_distance / args.right_speed,
            lateral=-args.right_speed,
        )
        time.sleep(0.5)

        node.get_logger().info("Moving forward toward interaction area 1")
        node.hold_velocity(
            args.forward_distance / args.forward_speed,
            forward=args.forward_speed,
        )
        time.sleep(0.5)

        node.get_logger().info("Turning to face I2 / exit side")
        node.hold_velocity(
            args.turn_angle / args.turn_speed,
            angular=-args.turn_speed,
        )

        node.set_mode("STAND_DEFAULT")
        node.get_logger().info("Track 1 initial movement finished")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
