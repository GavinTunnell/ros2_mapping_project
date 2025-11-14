#!/usr/bin/env python3
import sys, termios, tty, subprocess, threading, time, os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
WASD teleop  (/cmd_vel)
-----------------------
  W : forward      S : backward
  A : turn left    D : turn right
  Space : stop     Q : quit
  R : reset SLAM (kill + restart after 10 s)

  1/2 : linear speed -/+ (default 0.30 m/s)
  9/0 : angular speed -/+ (default 1.20 rad/s)
"""

# Path to your slam params
SLAM_PARAMS = os.path.expanduser(
    "~/Desktop/ros2_mapping_project/slam_params.yaml"
)

SLAM_CMD = [
    "ros2", "run", "slam_toolbox", "sync_slam_toolbox_node",
    "--ros-args", "--params-file", SLAM_PARAMS,
]


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
        self.slam_proc = None
        self.reset_in_progress = False

        self.get_logger().info(HELP)
        self.get_logger().info("Teleop ready. Make sure motor driver is running and listening to /cmd_vel.")

    # ----------------- SLAM control helpers -----------------

    def start_slam(self):
        # If already running, do nothing
        if self.slam_proc is not None and self.slam_proc.poll() is None:
            return
        self.get_logger().info(
            f"Starting slam_toolbox with params: {SLAM_PARAMS}"
        )
        try:
            self.slam_proc = subprocess.Popen(SLAM_CMD)
        except Exception as e:
            self.get_logger().error(f"Failed to start slam_toolbox: {e}")
            self.slam_proc = None

    def stop_slam(self):
        # Stop the process we launched (if any)
        if self.slam_proc is not None and self.slam_proc.poll() is None:
            self.get_logger().info("Stopping slam_toolbox (owned process)...")
            self.slam_proc.terminate()
            try:
                self.slam_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("slam_toolbox did not exit, killing...")
                self.slam_proc.kill()
            self.slam_proc = None

        # Backup: kill any sync_slam_toolbox_node processes
        try:
            subprocess.run(
                ["pkill", "-f", "slam_toolbox.*sync_slam_toolbox_node"],
                check=False,
            )
        except Exception as e:
            self.get_logger().warn(f"pkill backup failed: {e}")

    def reset_slam_with_delay(self, delay_sec: float = 10.0):
        if self.reset_in_progress:
            self.get_logger().warn("SLAM reset already in progress, ignoring.")
            return

        self.reset_in_progress = True

        def worker():
            try:
                self.get_logger().info(
                    f"SLAM reset requested. Stopping slam_toolbox and "
                    f"waiting {delay_sec:.1f} s before restart..."
                )
                # Stop SLAM immediately
                self.stop_slam()
                # Wait so you can reposition robot, etc.
                time.sleep(delay_sec)
                # Restart SLAM
                self.start_slam()
                self.get_logger().info("SLAM restarted. You can start mapping again.")
            finally:
                self.reset_in_progress = False

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    # ----------------- Teleop logic -----------------

    def _publish(self, vx, wz, note=''):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)
        if note:
            self.get_logger().info(note)

    def spin_keys(self):
        self._publish(0.0, 0.0, "ready: WASD, space=stop, r=reset SLAM, q=quit")
        while rclpy.ok():
            c = getch()
            if not c:
                continue

            c_lower = c.lower()
            self.get_logger().debug(f"Key pressed: {repr(c)}")

            if   c_lower == 'w':
                self._publish(self.lin, 0.0, f"cmd: forward  v={self.lin:.2f}")
            elif c_lower == 's':
                self._publish(-self.lin, 0.0, f"cmd: back     v={-self.lin:.2f}")
            elif c_lower == 'a':
                self._publish(0.0, self.ang, f"cmd: left     w={self.ang:.2f}")
            elif c_lower == 'd':
                self._publish(0.0, -self.ang, f"cmd: right    w={-self.ang:.2f}")
            elif c == ' ':
                self._publish(0.0, 0.0, "cmd: stop")

            # Speed tweaks
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

            # --- SLAM reset hotkey ---
            elif c_lower == 'r':
                self._publish(0.0, 0.0, "cmd: stop (SLAM reset requested)")
                self.reset_slam_with_delay(delay_sec=10.0)

            elif c_lower == 'q':
                self.get_logger().info("Quit requested, stopping teleop.")
                break


def main():
    rclpy.init()
    node = WasdTeleop()
    try:
        node.spin_keys()
    finally:
        # final stop for safety
        msg = Twist()
        node.pub.publish(msg)
        node.stop_slam()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
