#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time


class OdometryChecker(Node):
    def __init__(self, topic):
        super().__init__("odometry_checker")
        self.topic = topic
        self.received_data = False
        self.message_type = None
        
        # Generic subscription to check message type
        from rclpy.subscription import Subscription
        
        def callback(msg):
            self.received_data = True
            self.message_type = type(msg).__name__
            self.get_logger().info(f"Message type: {self.message_type}")
            self.get_logger().info(f"Message:\n{msg}")
        
        self.subscription = self.create_subscription(
            object,  # Accept any message type
            self.topic,
            callback,
            10
        )


def main():
    topic = "/aima/mc/leg_odometry"
    
    print(f"Checking topic: {topic}")
    
    rclpy.init()
    node = OdometryChecker(topic)
    
    print("Waiting for data (10 seconds)...")
    start = time.time()
    
    try:
        while (time.time() - start) < 10:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.received_data:
                print(f"✓ Successfully received data from {topic}")
                print(f"  Message type: {node.message_type}")
                break
    except KeyboardInterrupt:
        pass
    finally:
        if not node.received_data:
            print(f"✗ No data received from {topic}")
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
