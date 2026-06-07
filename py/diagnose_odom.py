#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import time


class OdomDiagnostic(Node):
    def __init__(self):
        super().__init__("odom_diagnostic")
        self.odom_data = {}
        self.subs = []  # Use different name to avoid conflicts
        
        # Try different QoS profiles
        qos_profiles = {
            "BEST_EFFORT": QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ),
            "RELIABLE": QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            ),
        }
        
        topics = [
            "/aima/mc/leg_odometry",
            "/aima/hal/odom/state",
        ]
        
        for topic in topics:
            for qos_name, qos_profile in qos_profiles.items():
                def make_callback(t, q):
                    def callback(msg):
                        key = f"{t}_{q}"
                        if key not in self.odom_data:
                            self.get_logger().info(f"✓ Received data from {t} with {q} QoS")
                        self.odom_data[key] = msg
                    return callback
                
                sub_name = f"{topic}_{qos_name}"
                try:
                    sub = self.create_subscription(
                        Odometry,
                        topic,
                        make_callback(topic, qos_name),
                        qos_profile
                    )
                    self.subs.append((sub_name, sub))
                except Exception as e:
                    self.get_logger().error(f"Failed to subscribe to {topic} with {qos_name}: {e}")


def main():
    rclpy.init()
    node = OdomDiagnostic()
    
    print("\nTrying to receive odometry data with different QoS settings...")
    print("Waiting 10 seconds...\n")
    
    start = time.time()
    while (time.time() - start) < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.05)
    
    print("\n" + "="*60)
    if node.odom_data:
        print(f"✓ Successfully received data from:")
        for key in node.odom_data.keys():
            print(f"  - {key}")
    else:
        print("✗ No odometry data received from any topic/QoS combination")
    print("="*60)
    
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
