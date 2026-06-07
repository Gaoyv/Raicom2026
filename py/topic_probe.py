#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node


class TopicProbe(Node):
    def __init__(self):
        super().__init__("topic_probe")

    def collect_topics(self, wait_sec):
        deadline = time.monotonic() + wait_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)
        return self.get_topic_names_and_types()

    def print_topics(self, topics, keyword=None):
        print("\n=== All ROS 2 Topics ===")
        if not topics:
            print("No topics found")
            return

        for name, types in sorted(topics):
            type_text = ", ".join(types)
            print(f"{name:<45} {type_text}")

        if keyword:
            keyword_lower = keyword.lower()
            print(f"\n=== Filtered Topics: {keyword} ===")
            matched = []
            for name, types in sorted(topics):
                haystack = " ".join([name] + list(types)).lower()
                if keyword_lower in haystack:
                    matched.append((name, types))

            if not matched:
                print("No matching topics found")
            else:
                for name, types in matched:
                    type_text = ", ".join(types)
                    print(f"{name:<45} {type_text}")

        print("\n=== Recommended commands ===")
        print("ros2 topic list")
        print("ros2 topic list -t")
        print("ros2 topic info /your/topic")
        print("ros2 interface show <msg_type>")
        print("ros2 topic echo /your/topic")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Probe all ROS 2 topic names and message types"
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=2.0,
        help="Seconds to wait for discovery before printing topics",
    )
    parser.add_argument(
        "--filter",
        type=str,
        default="",
        help="Optional keyword filter for topic name or type",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = TopicProbe()

    try:
        topics = node.collect_topics(args.wait)
        node.print_topics(topics, keyword=args.filter or None)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
