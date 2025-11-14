#!/usr/bin/env python3
import sys
import time
import termios
import tty

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class SlamResetNode(Node):
    def __init__(self):
        super().__init__('slam_reset_node')
        self.cli = self.create_client(Empty, '/slam_toolbox/clear')
        self.get_logger().info("Waiting for /slam_toolbox/clear service...")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/slam_toolbox/clear not available yet...')
        self.get_logger().info("Connected to /slam_toolbox/clear.")

    def reset_map(self):
        req = Empty.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("SLAM map cleared.")
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")


def main():
    rclpy.init()
    node = SlamResetNode()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    cooldown = 1.0   # seconds
    last_reset = 0.0

    print("")
    print("=== SLAM reset keyboard helper ===")
    print("Press 'r' to clear the slam_toolbox map.")
    print(f"Resets allowed at most once every {cooldown:.0f} seconds.")
    print("Press 'q' or Ctrl+C to quit.")
    print("===================================")

    try:
        tty.setcbreak(fd)  # raw-ish keyboard
        while rclpy.ok():
            ch = sys.stdin.read(1)
            now = time.time()

            if ch == 'r':
                dt = now - last_reset
                if dt < cooldown:
                    remaining = cooldown - dt
                    print(f"[cooldown] Wait {remaining:.1f} s before next reset.")
                    continue

                print("[key] 'r' pressed -> clearing SLAM map...")
                node.reset_map()
                last_reset = time.time()
                print(f"[info] You can reset again in {cooldown:.0f} s.")

            elif ch in ('q', '\x03'):  # 'q' or Ctrl+C
                print("[exit] Quitting slam_reset_key.")
                break

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
