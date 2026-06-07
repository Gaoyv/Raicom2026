#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
import time


class TopicLister(Node):
    def __init__(self):
        super().__init__("topic_lister")

    def list_topics(self):
        """List all available ROS2 topics"""
        from rclpy.executors import SingleThreadedExecutor
        
        # Get topic info from ros2 service
        try:
            import subprocess
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                print("\n=== Available ROS2 Topics ===")
                for topic in topics:
                    if topic:
                        print(f"  {topic}")
                
                # Filter odometry-related topics
                print("\n=== Odometry-related Topics ===")
                odom_topics = [t for t in topics if 'odom' in t.lower()]
                if odom_topics:
                    for topic in odom_topics:
                        print(f"  {topic}")
                else:
                    print("  No odometry topics found")
                
                # Filter locomotion-related topics
                print("\n=== Locomotion/Motion-related Topics ===")
                motion_topics = [t for t in topics if any(keyword in t.lower() for keyword in ['locomotion', 'motion', 'velocity', 'pose'])]
                if motion_topics:
                    for topic in motion_topics:
                        print(f"  {topic}")
                else:
                    print("  No motion-related topics found")
                    
            else:
                print("Error listing topics:", result.stderr)
        except Exception as e:
            print(f"Error: {e}")


def main():
    rclpy.init()
    node = TopicLister()
    
    try:
        node.list_topics()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
