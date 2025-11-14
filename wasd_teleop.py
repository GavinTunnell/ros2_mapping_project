#!/usr/bin/env python3
import sys, termios, tty, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

HELP = """
WASD teleop  (/cmd_vel)
-----------------------
  W : forward      S : backward
  A : turn left    D : turn right
  Space : stop     Q : quit

  1/2 : linear speed -/+ (default 0.30 m/s)
  9/0 : angular speed -/+ (default 1.20 rad/s)

  R : reset SLAM map (/slam_toolbox/clear), 10s cooldown
"""

def getch():
    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        return ''  # ignore if not a TTY
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lin = 0.30
        self.ang = 1.20

        # SLAM reset client
        self.slam_client = self.create_client(Empty, '/slam_toolbox/clear')
        self.reset_cooldown = 10.0  # seconds
        self.last_reset_time = 0.0

        if not self.slam_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "/slam_toolbox/clear service not available yet. "
                "R will do nothing until slam_toolbox is running."
            )
        else:
            self.get_logger().info("Connected to /slam_toolbox/clear.")

        self.get_logger().info(HELP)

    def _publish(self, vx, wz, note=''):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)
        if note:
            self.get_logger().info(note)

    def _reset_slam(self):
        now = time.time()
        dt = now - self.last_reset_time

        if not self.slam_client.service_is_ready():
            self.get_logger().warn("SLAM reset requested but /slam_toolbox/clear is not available.")
            return

        if dt < self.reset_cooldown:
            remaining = self.reset_cooldown - dt
            self.get_logger().info(
                f"SLAM reset on cooldown. Wait {remaining:.1f} s before next reset."
            )
            return

        self.get_logger().info("Resetting SLAM map via /slam_toolbox/clear...")
        req = Empty.Request()
        future = self.slam_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            self.last_reset_time = time.time()
            self.get_logger().info(
                "SLAM map cleared. Drive the robot to build a new map."
            )
        else:
            self.get_logger().error(
                f"SLAM reset failed: {future.exception()}"
            )

    def spin_keys(self):
        self._publish(0.0, 0.0, "ready: WASD, space=stop, r=reset SLAM, q=quit")
        while rclpy.ok():
            c = getch().lower()
            if not c:
                continue

            if   c == 'w':
                self._publish(self.lin, 0.0, f"cmd: forward  v={self.lin:.2f}")
            elif c == 's':
                self._publish(-self.lin, 0.0, f"cmd: back     v={-self.lin:.2f}")
            elif c == 'a':
                self._publish(0.0,  self.ang, f"cmd: left     w={ self.ang:.2f}")
            elif c == 'd':
                self._publish(0.0, -self.ang, f"cmd: right    w={-self.ang:.2f}")
            elif c == ' ':
                self._publish(0.0, 0.0, "cmd: stop")
            elif c == '1':
                self.lin = max(0.05, round(self.lin - 0.05, 3))
                self.get_logger().info(f"linear = {self.lin:.2f} m/s")
            elif c == '2':
                self.lin = min(1.50, round(self.lin + 0.05, 3))
                self.get_logger().info(f"linear = {self.lin:.2f} m/s")
            elif c == '9':
                self.ang = max(0.20, round(self.ang - 0.10, 3))
                self.get_logger().info(f"angular = {self.ang:.2f} rad/s")
            elif c == '0':
                self.ang = min(4.00, round(self.ang + 0.10, 3))
                self.get_logger().info(f"angular = {self.ang:.2f} rad/s")
            elif c == 'r':
                self._reset_slam()
            elif c == 'q':
                break

def main():
    rclpy.init()
    node = WasdTeleop()
    try:
        node.spin_keys()
    finally:
        # send a final stop for safety
        msg = Twist()
        node.pub.publish(msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
