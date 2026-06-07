#!/usr/bin/env python3

import argparse
import math
import signal
import sys
import time
from collections import deque

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
from sensor_msgs.msg import Imu


class ImuWalkFixedDistance(Node):
    def __init__(
        self,
        imu_topic,
        odom_topic,
        distance,
        forward_speed,
        distance_kp,
        heading_kp,
        pose_log_interval,
        imu_log_interval,
    ):
        super().__init__("imu_walk_fixed_distance")
        self.publisher = self.create_publisher(
            McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10
        )
        self.mode_client = self.create_client(
            SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction"
        )
        self.input_client = self.create_client(
            SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource"
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.on_imu, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.on_odom, sensor_qos
        )

        self.source = "node"
        self.seq = 0
        self.imu_topic = imu_topic
        self.odom_topic = odom_topic
        self.target_distance = float(distance)
        self.forward_speed = float(forward_speed)
        self.distance_kp = float(distance_kp)
        self.heading_kp = float(heading_kp)
        self.pose_log_interval = float(pose_log_interval)
        self.imu_log_interval = float(imu_log_interval)

        self.max_heading_correction = 0.35
        self.max_distance_speed = self.forward_speed
        self.min_distance_speed = 0.10
        self.position_tolerance = 0.05

        self.imu_ready = False
        self.odom_ready = False

        self.x = 0.0
        self.y = 0.0
        self.odom_yaw = 0.0

        self.imu_yaw = 0.0
        self.imu_orientation = (0.0, 0.0, 0.0, 1.0)
        self.imu_angular_velocity = (0.0, 0.0, 0.0)
        self.imu_linear_acceleration = (0.0, 0.0, 0.0)

        self.last_pose_log_time = 0.0
        self.last_imu_log_time = 0.0
        self.arrivals = deque()

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def on_imu(self, msg):
        q = msg.orientation
        self.imu_orientation = (q.x, q.y, q.z, q.w)
        self.imu_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.imu_angular_velocity = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        self.imu_linear_acceleration = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        self.imu_ready = True
        self.update_arrivals()

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        q = pose.orientation
        self.odom_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_ready = True

    def update_arrivals(self):
        now = self.get_clock().now()
        self.arrivals.append(now)
        while self.arrivals and (now - self.arrivals[0]).nanoseconds * 1e-9 > 1.0:
            self.arrivals.popleft()

    def get_fps(self):
        return float(len(self.arrivals))

    def wait_for_sensors(self, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.imu_ready and self.odom_ready:
                self.get_logger().info(
                    f"IMU ready on {self.imu_topic}; Odom ready on {self.odom_topic}: "
                    f"x={self.x:.3f}, y={self.y:.3f}, odom_yaw={math.degrees(self.odom_yaw):.1f} deg, "
                    f"imu_yaw={math.degrees(self.imu_yaw):.1f} deg"
                )
                return True
        if not self.imu_ready:
            self.get_logger().error(f"Timed out waiting for {self.imu_topic}")
        if not self.odom_ready:
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

    def set_mode(self, action_desc, source="node"):
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
        req.source = source
        cmd = McActionCommand()
        cmd.action_desc = action_desc
        req.command = cmd

        for i in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            future = self.mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"Trying to set {action_desc}... [{i}]")

        if not future.done() or future.result() is None:
            self.get_logger().error("SetMcAction call failed or timed out")
            return False

        response = future.result()
        if response.response.status.value == CommonState.SUCCESS:
            self.get_logger().info(f"{action_desc} set successfully")
            return True

        self.get_logger().error(
            f"Failed to set {action_desc}: {response.response.message}"
        )
        return False

    def set_locomotion_mode(self):
        return self.set_mode("LOCOMOTION_DEFAULT")

    def make_header(self):
        header = MessageHeader()
        now = self.get_clock().now().to_msg()
        header.stamp = now
        header.meas_stamp = now
        header.frame_id = "base_link"
        header.sequence = self.seq
        self.seq += 1
        return header

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

    def maybe_log_pose(self, traveled, remaining):
        now = time.monotonic()
        if now - self.last_pose_log_time >= self.pose_log_interval:
            self.last_pose_log_time = now
            self.get_logger().info(
                f"Pose: x={self.x:.3f}, y={self.y:.3f}, traveled={traveled:.3f} m, "
                f"remaining={remaining:.3f} m, odom_yaw={math.degrees(self.odom_yaw):.1f} deg"
            )

    def maybe_log_imu(self):
        now = time.monotonic()
        if now - self.last_imu_log_time >= self.imu_log_interval:
            self.last_imu_log_time = now
            ox, oy, oz, ow = self.imu_orientation
            gx, gy, gz = self.imu_angular_velocity
            ax, ay, az = self.imu_linear_acceleration
            self.get_logger().info(
                f"IMU: yaw={math.degrees(self.imu_yaw):.1f} deg, "
                f"orientation=[{ox:.4f}, {oy:.4f}, {oz:.4f}, {ow:.4f}], "
                f"angular_velocity=[{gx:.4f}, {gy:.4f}, {gz:.4f}], "
                f"linear_accel=[{ax:.4f}, {ay:.4f}, {az:.4f}], fps={self.get_fps():.1f}"
            )

    def drive_fixed_distance(self):
        start_x = self.x
        start_y = self.y
        target_heading = self.imu_yaw if self.imu_ready else self.odom_yaw
        self.last_pose_log_time = 0.0
        self.last_imu_log_time = 0.0

        self.get_logger().info(
            "Starting fixed-distance walk: distance=%.2f m, velocity=%.2f m/s. "
            "Distance stop uses odometry; IMU is used for heading reference/logging."
            % (self.target_distance, self.forward_speed)
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            traveled = math.hypot(self.x - start_x, self.y - start_y)
            remaining = self.target_distance - traveled
            if remaining <= self.position_tolerance:
                break

            current_heading = self.imu_yaw if self.imu_ready else self.odom_yaw
            heading_error = self.normalize_angle(target_heading - current_heading)
            angular = max(
                -self.max_heading_correction,
                min(self.max_heading_correction, self.heading_kp * heading_error),
            )
            forward = max(
                self.min_distance_speed,
                min(self.max_distance_speed, self.distance_kp * remaining),
            )

            self.publish_velocity(forward=forward, angular=angular)
            self.maybe_log_pose(traveled, remaining)
            self.maybe_log_imu()
            time.sleep(0.02)

        self.stop()
        final_traveled = math.hypot(self.x - start_x, self.y - start_y)
        self.get_logger().info(
            f"Finished fixed-distance walk: traveled={final_traveled:.3f} m, "
            f"final x={self.x:.3f}, y={self.y:.3f}"
        )


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
        description="Subscribe to IMU and drive a fixed distance using odometry for stop control"
    )
    parser.add_argument(
        "--imu-topic",
        type=str,
        default="/aima/hal/imu/chest/state",
        help="IMU topic used for orientation and debug logging",
    )
    parser.add_argument(
        "--odom-topic",
        type=str,
        default="/aima/hal/odom/state",
        help="Odometry topic used for fixed-distance stop control",
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=10.0,
        help="Target forward distance in meters",
    )
    parser.add_argument(
        "--velocity",
        type=float,
        default=0.25,
        help="Maximum forward velocity in m/s",
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
        default=1.0,
        help="Proportional gain for IMU-based heading hold",
    )
    parser.add_argument(
        "--pose-log-interval",
        type=float,
        default=0.5,
        help="Seconds between odometry pose logs while walking",
    )
    parser.add_argument(
        "--imu-log-interval",
        type=float,
        default=1.0,
        help="Seconds between IMU debug logs while walking",
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
    return parser.parse_args()


def main():
    global _global_node

    args = _parse_args()
    rclpy.init()
    node = ImuWalkFixedDistance(
        args.imu_topic,
        args.odom_topic,
        args.distance,
        args.velocity,
        args.distance_kp,
        args.heading_kp,
        args.pose_log_interval,
        args.imu_log_interval,
    )
    _global_node = node

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        if not node.set_mode("STAND_DEFAULT"):
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
            return
        if not node.set_locomotion_mode():
            return
        if not node.wait_for_sensors():
            return

        node.drive_fixed_distance()
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
