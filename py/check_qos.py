#!/usr/bin/env python3

import subprocess
import sys


def get_detailed_topic_info(topic_name):
    """Get detailed QoS info for a topic"""
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic_name, "-v"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            print(f"\n{'='*60}")
            print(f"Topic: {topic_name}")
            print('='*60)
            print(result.stdout)
        else:
            print(f"Error: {result.stderr}")
    except Exception as e:
        print(f"Error: {e}")


def main():
    topics = [
        "/aima/mc/leg_odometry",
        "/aima/hal/odom/state",
    ]
    
    for topic in topics:
        get_detailed_topic_info(topic)


if __name__ == "__main__":
    main()
