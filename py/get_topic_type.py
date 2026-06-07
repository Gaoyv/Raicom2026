#!/usr/bin/env python3

import subprocess
import sys


def get_topic_type(topic_name):
    """Get the message type of a topic"""
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic_name],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            output = result.stdout
            print(f"Topic info for {topic_name}:")
            print(output)
            
            # Extract message type
            for line in output.split('\n'):
                if 'Type:' in line:
                    msg_type = line.split('Type:')[1].strip()
                    print(f"\nMessage type: {msg_type}")
                    return msg_type
        else:
            print(f"Error: {result.stderr}")
    except Exception as e:
        print(f"Error: {e}")
    
    return None


def main():
    topics = [
        "/aima/mc/leg_odometry",
        "/aima/hal/odom/state",
    ]
    
    for topic in topics:
        print(f"\n{'='*60}")
        get_topic_type(topic)


if __name__ == "__main__":
    main()
