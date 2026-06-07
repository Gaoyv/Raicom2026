import argparse
import sys

import rclpy
from rclpy.node import Node

from aimdk_msgs.srv import SetMcAction
from aimdk_msgs.msg import RequestHeader, CommonState, McActionCommand


MODES = {
    "PD": ("PASSIVE_DEFAULT", "joints with zero torque"),
    "DD": ("DAMPING_DEFAULT", "joints in damping mode"),
    "JD": ("JOINT_DEFAULT", "Position Control Stand (joints locked)"),
    "SD": ("STAND_DEFAULT", "Stable Stand (auto-balance)"),
    "LD": ("LOCOMOTION_DEFAULT", "locomotion mode (walk or run)"),
    "US": ("UPPERBODY_REMOTE_SPLIT", "upper body remote control"),
    "HO": ("HEAD_ONLY", "head only control"),
}


class SetModeClient(Node):
    def __init__(self):
        super().__init__("set_mode_client")
        self.client = self.create_client(SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction")

    def set_mode(self, action_name: str, source: str = "rc") -> bool:
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service not available")
            return False

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = source

        cmd = McActionCommand()
        cmd.action_desc = action_name
        req.command = cmd

        for i in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                break
            self.get_logger().info(f"Retrying... [{i}]")

        response = future.result()
        if response is None:
            self.get_logger().error("Service call failed")
            return False

        if response.response.status.value == CommonState.SUCCESS:
            self.get_logger().info("Mode set successfully")
            return True
        else:
            self.get_logger().error(f"Failed: {response.response.message}")
            return False


def list_modes():
    print(f"{'Abbr':<6} {'Mode':<25} {'Description'}")
    for abbr, (name, desc) in MODES.items():
        print(f"{abbr:<6} {name:<25} {desc}")


def main(args=None):
    parser = argparse.ArgumentParser(description="Set robot mode")
    parser.add_argument("mode", nargs="?", help="Mode abbreviation (e.g., LD, SD)")
    parser.add_argument("--list", action="store_true", help="List available modes")
    parsed = parser.parse_args(args)

    if parsed.list or parsed.mode is None:
        list_modes()
        if parsed.mode is None:
            abbr = input("Enter mode abbreviation: ").strip().upper()
        else:
            return
    else:
        abbr = parsed.mode.upper()

    source = "rc"

    mode_info = MODES.get(abbr)
    if not mode_info:
        print(f"Unknown mode: {abbr}")
        list_modes()
        sys.exit(1)

    rclpy.init()
    node = None
    try:
        node = SetModeClient()
        ok = node.set_mode(mode_info[0], source)
        sys.exit(0 if ok else 1)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
