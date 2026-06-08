#!/usr/bin/env python3

import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

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
from nav_msgs.msg import Odometry


TASK_REQUEST_START = 1
TASK_REQUEST_STOP = 2
TASK_TYPE_PLANNING_NAVI_TO_POSE_2D = 2
PNC_MODE_NORMAL = 0
SCENE_START_X = -1.5
SCENE_START_Y = -1.5
SCENE_START_YAW = 1.57


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
        self.pose = None
        self.pose_topic = None

        lidar_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        reliable_lidar_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Odometry, "/slam/lidar_loc",
            lambda msg: self.odometry_callback(msg, "/slam/lidar_loc"),
            qos_profile=lidar_qos)
        self.create_subscription(
            Odometry, "/slam/lidar_odom",
            lambda msg: self.odometry_callback(msg, "/slam/lidar_odom"),
            qos_profile=reliable_lidar_qos)

    def odometry_callback(self, msg, topic):
        pose = msg.pose.pose
        self.pose = (
            pose.position.x,
            pose.position.y,
            quaternion_to_yaw(pose.orientation),
        )
        self.pose_topic = topic

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

    def wait_for_pose(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None:
                self.get_logger().info(
                    "Using localization from %s: x=%.3f y=%.3f yaw=%.3f",
                    self.pose_topic, self.pose[0], self.pose[1], self.pose[2])
                return True
        self.get_logger().warning("No localization pose received")
        return False

    def drive_to_pose_closed_loop(
            self, target_x, target_y, target_yaw, pos_tol, yaw_tol,
            max_forward, max_lateral, max_angular, timeout,
            no_progress_timeout, progress_epsilon, axis_switch_tolerance):
        deadline = time.monotonic() + timeout
        last_log = 0.0
        best_distance = None
        best_distance_time = time.monotonic()

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.pose is None:
                self.stop(repeats=1)
                continue

            x, y, yaw = self.pose
            dx = target_x - x
            dy = target_y - y
            distance = math.hypot(dx, dy)

            if distance <= pos_tol:
                self.stop()
                break

            now = time.monotonic()
            if best_distance is None or distance < best_distance - progress_epsilon:
                best_distance = distance
                best_distance_time = now
            elif now - best_distance_time > no_progress_timeout:
                self.stop()
                self.get_logger().error(
                    "No progress toward I1 for %.1fs. "
                    "Stopping to avoid pushing into a wall. "
                    "Current dist=%.3f, best dist=%.3f",
                    no_progress_timeout, distance, best_distance)
                return False

            forward_error = math.cos(yaw) * dx + math.sin(yaw) * dy
            lateral_error = -math.sin(yaw) * dx + math.cos(yaw) * dy

            forward = 0.0
            lateral = 0.0
            if abs(lateral_error) > axis_switch_tolerance:
                lateral = command_from_error(
                    lateral_error, gain=0.35, min_abs=0.20, max_abs=max_lateral)
            elif abs(forward_error) > axis_switch_tolerance:
                forward = command_from_error(
                    forward_error, gain=0.35, min_abs=0.20, max_abs=max_forward)
            else:
                forward = command_from_error(
                    forward_error, gain=0.25, min_abs=0.20, max_abs=max_forward)
                lateral = command_from_error(
                    lateral_error, gain=0.25, min_abs=0.20, max_abs=max_lateral)

            self.publish_velocity(forward=forward, lateral=lateral)
            if now - last_log > 1.0:
                self.get_logger().info(
                    "Approaching I1: dist=%.3f forward_err=%.3f "
                    "lateral_err=%.3f cmd=(%.2f, %.2f)",
                    distance, forward_error, lateral_error, forward, lateral)
                last_log = now
            time.sleep(0.02)
        else:
            self.get_logger().warning("Position control timed out")

        self.stop()
        deadline = time.monotonic() + timeout
        last_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.pose is None:
                self.stop(repeats=1)
                continue

            yaw_error = normalize_angle(target_yaw - self.pose[2])
            if abs(yaw_error) <= yaw_tol:
                self.stop()
                return True

            angular = command_from_error(
                yaw_error, gain=0.65, min_abs=0.10, max_abs=max_angular)
            self.publish_velocity(angular=angular)
            now = time.monotonic()
            if now - last_log > 1.0:
                self.get_logger().info(
                    "Aligning to I2: yaw_error=%.3f", yaw_error)
                last_log = now
            time.sleep(0.02)

        self.get_logger().warning("Yaw control timed out")
        self.stop()
        return False

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


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, low, high):
    return max(low, min(high, value))


def command_from_error(error, gain, min_abs, max_abs):
    if abs(error) < 1e-6:
        return 0.0
    raw = clamp(gain * error, -max_abs, max_abs)
    if 0.0 < abs(raw) < min_abs:
        raw = math.copysign(min_abs, raw)
    return raw


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
        choices=("closed-loop", "pnc", "velocity"),
        default="closed-loop",
        help="Use odometry closed-loop control, PNC, or timed velocity fallback.")
    parser.add_argument("--scene-start-x", type=float, default=SCENE_START_X)
    parser.add_argument("--scene-start-y", type=float, default=SCENE_START_Y)
    parser.add_argument("--scene-start-yaw", type=float, default=SCENE_START_YAW)
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
    parser.add_argument("--pose-timeout", type=float, default=5.0)
    parser.add_argument("--closed-loop-timeout", type=float, default=45.0)
    parser.add_argument("--position-tolerance", type=float, default=0.08)
    parser.add_argument("--yaw-tolerance", type=float, default=0.08)
    parser.add_argument("--closed-loop-forward-speed", type=float, default=0.20)
    parser.add_argument("--closed-loop-lateral-speed", type=float, default=0.20)
    parser.add_argument("--closed-loop-angular-speed", type=float, default=0.25)
    parser.add_argument("--axis-switch-tolerance", type=float, default=0.12)
    parser.add_argument("--no-progress-timeout", type=float, default=2.5)
    parser.add_argument("--progress-epsilon", type=float, default=0.03)
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
        if args.control_mode == "closed-loop":
            if not node.register_input_source():
                return

            if node.wait_for_pose(args.pose_timeout):
                start_x, start_y, start_yaw = node.pose
                scene_dx = args.target_x - args.scene_start_x
                scene_dy = args.target_y - args.scene_start_y
                frame_yaw = normalize_angle(start_yaw - args.scene_start_yaw)
                target_x = (
                    start_x
                    + math.cos(frame_yaw) * scene_dx
                    - math.sin(frame_yaw) * scene_dy
                )
                target_y = (
                    start_y
                    + math.sin(frame_yaw) * scene_dx
                    + math.cos(frame_yaw) * scene_dy
                )
                target_yaw = normalize_angle(
                    start_yaw + args.target_yaw - args.scene_start_yaw)

                node.get_logger().info(
                    "Reset pose: x=%.3f y=%.3f yaw=%.3f; "
                    "scene delta to I1=(%.3f, %.3f); "
                    "closed-loop target: x=%.3f y=%.3f yaw=%.3f",
                    start_x, start_y, start_yaw,
                    scene_dx, scene_dy,
                    target_x, target_y, target_yaw)
                ok = node.drive_to_pose_closed_loop(
                    target_x,
                    target_y,
                    target_yaw,
                    args.position_tolerance,
                    args.yaw_tolerance,
                    args.closed_loop_forward_speed,
                    args.closed_loop_lateral_speed,
                    args.closed_loop_angular_speed,
                    args.closed_loop_timeout,
                    args.no_progress_timeout,
                    args.progress_epsilon,
                    args.axis_switch_tolerance,
                )
                node.set_mode("STAND_DEFAULT")
                if ok:
                    node.get_logger().info(
                        "Closed-loop movement finished at I1 target pose")
                return

            node.get_logger().warning(
                "Localization unavailable; falling back to timed velocity control.")
            use_velocity_fallback = True

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
