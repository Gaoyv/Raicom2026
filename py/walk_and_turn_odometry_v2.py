#!/usr/bin/env python3

import math
import signal
import sys
import time
import subprocess

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader, RequestHeader, CommonState, McActionCommand
from aimdk_msgs.srv import SetMcInputSource, SetMcAction


def get_message_type_from_topic(topic_name):
    """Dynamically determine and import the message type for a topic"""
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic_name, "-v"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            output = result.stdout
            # Look for "Type: package/MessageType" format
            for line in output.split('\n'):
                if 'Type:' in line:
                    msg_type_str = line.split('Type:')[1].strip()
                    print(f"Found message type: {msg_type_str}")
                    
                    # Parse package and message name
                    if '/' in msg_type_str:
                        package, msg_name = msg_type_str.split('/')
                        try:
                            # Try to import the message type
                            module = __import__(f'{package}.msg', fromlist=[msg_name])
                            msg_class = getattr(module, msg_name)
                            return msg_class
                        except Exception as e:
                            print(f"Could not import {msg_type_str}: {e}")
    except Exception as e:
        print(f"Error getting topic info: {e}")
    
    return None


class WalkAndTurnWithOdometry(Node):
    def __init__(self):
        super().__init__("walk_and_turn_odometry")
        self.publisher = self.create_publisher(
            McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10
        )
        self.client = self.create_client(
            SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource"
        )
        self.action_client = self.create_client(
            SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction"
        )
        
        # Try to get the correct message type
        msg_type = get_message_type_from_topic("/aima/mc/leg_odometry")
        if msg_type:
            self.get_logger().info(f"Using message type: {msg_type}")
            self.odom_subscriber = self.create_subscription(
                msg_type, "/aima/mc/leg_odometry", self.odom_callback, 10
            )
        else:
            # Fall back to nav_msgs/Odometry
            self.get_logger().warning("Could not determine message type, trying nav_msgs/Odometry")
            try:
                from nav_msgs.msg import Odometry
                self.odom_subscriber = self.create_subscription(
                    Odometry, "/aima/mc/leg_odometry", self.odom_callback, 10
                )
            except:
                self.get_logger().error("Could not subscribe to odometry topic")
                raise

        self.timer = None
        self.forward_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Odometry data
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.odom_received = False
        self.message_type = None

    def odom_callback(self, msg):
        """Callback for odometry data - handles different message formats"""
        if not self.message_type:
            self.message_type = type(msg).__name__
            self.get_logger().info(f"Odometry message type: {self.message_type}")
        
        try:
            # Try nav_msgs/Odometry format
            if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                pose = msg.pose.pose
                self.current_x = pose.position.x
                self.current_y = pose.position.y
                
                # Extract yaw from quaternion
                q = pose.orientation
                self.current_yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                self.odom_received = True
            
            # Try alternative formats (e.g., different message structure)
            elif hasattr(msg, 'position') and hasattr(msg.position, 'x'):
                self.current_x = msg.position.x
                self.current_y = msg.position.y if hasattr(msg.position, 'y') else 0.0
                
                if hasattr(msg, 'orientation'):
                    q = msg.orientation
                    self.current_yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    )
                self.odom_received = True
            
            else:
                if not self.odom_received:
                    self.get_logger().warning(f"Could not parse odometry message. Message attributes: {[attr for attr in dir(msg) if not attr.startswith('_')]}")
        
        except Exception as e:
            self.get_logger().error(f"Error parsing odometry: {e}")

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

    def walk_forward_distance(self, distance, velocity=0.5):
        """Walk forward a specific distance (in meters)"""
        if not self.odom_received:
            self.get_logger().error("Odometry not received yet")
            return False
        
        # Record starting position
        self.start_x = self.current_x
        self.start_y = self.current_y
        
        self.get_logger().info(f"Starting position: ({self.start_x:.3f}, {self.start_y:.3f})")
        self.get_logger().info(f"Walking forward {distance:.2f}m at velocity {velocity:.2f} m/s")
        
        self.forward_velocity = velocity
        self.angular_velocity = 0.0
        
        # Walk until target distance is reached
        timeout_counter = 0
        max_timeout = 300  # 6 seconds timeout at 50Hz
        
        while timeout_counter < max_timeout:
            # Calculate distance traveled
            delta_x = self.current_x - self.start_x
            delta_y = self.current_y - self.start_y
            traveled_distance = math.sqrt(delta_x**2 + delta_y**2)
            
            if traveled_distance >= distance * 0.95:  # 95% tolerance
                self.get_logger().info(f"Target distance reached: {traveled_distance:.3f}m")
                break
            
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.001)
            timeout_counter += 1
        
        if timeout_counter >= max_timeout:
            self.get_logger().warning(f"Timeout: traveled {traveled_distance:.3f}m (target: {distance:.2f}m)")
        
        self.forward_velocity = 0.0
        return True

    def turn_clockwise_angle(self, angle_deg, angular_velocity=-0.8):
        """Turn clockwise by specified angle (in degrees)"""
        if not self.odom_received:
            self.get_logger().error("Odometry not received yet")
            return False
        
        angle_rad = math.radians(angle_deg)
        self.start_yaw = self.current_yaw
        
        self.get_logger().info(f"Starting yaw: {math.degrees(self.start_yaw):.2f} degrees")
        self.get_logger().info(f"Turning clockwise {angle_deg:.2f} degrees")
        
        self.forward_velocity = 0.0
        self.angular_velocity = angular_velocity
        
        # Turn until target angle is reached
        timeout_counter = 0
        max_timeout = 300  # 6 seconds timeout at 50Hz
        
        while timeout_counter < max_timeout:
            # Calculate angle turned (clockwise is negative)
            delta_yaw = self.start_yaw - self.current_yaw
            
            # Handle yaw wrapping (-pi to pi)
            while delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            while delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            
            if delta_yaw >= angle_rad * 0.95:  # 95% tolerance
                self.get_logger().info(f"Target angle reached: {math.degrees(delta_yaw):.2f} degrees")
                break
            
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.001)
            timeout_counter += 1
        
        if timeout_counter >= max_timeout:
            self.get_logger().warning(f"Timeout: turned {math.degrees(delta_yaw):.2f} degrees (target: {angle_deg:.2f})")
        
        self.angular_velocity = 0.0
        return True


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
    node = WalkAndTurnWithOdometry()
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

    # Wait for odometry data
    node.get_logger().info("Waiting for odometry data...")
    for i in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom_received:
            node.get_logger().info("Odometry data received")
            break
        time.sleep(0.1)

    if not node.odom_received:
        node.get_logger().error("No odometry data received, exiting")
        if rclpy.ok():
            rclpy.shutdown()
        return

    node.start_publish()
    time.sleep(0.5)  # Wait for publisher to stabilize

    # Step 1: Walk forward 9 steps
    # Assuming step length is 0.3m, 9 steps = 2.7m
    step_length = 0.3  # meters per step
    distance_1 = 9 * step_length
    node.walk_forward_distance(distance_1, velocity=0.5)
    time.sleep(0.5)

    # Step 2: Turn clockwise 90 degrees
    node.turn_clockwise_angle(90, angular_velocity=-0.8)
    time.sleep(0.5)

    # Step 3: Walk forward 3 steps
    distance_2 = 3 * step_length
    node.walk_forward_distance(distance_2, velocity=0.5)

    node.stop()
    node.get_logger().info("All movements completed, robot stopped")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
