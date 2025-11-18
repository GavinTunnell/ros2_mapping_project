#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

try:
    import Jetson.GPIO as GPIO
    _HAS_GPIO = True
except Exception:
    _HAS_GPIO = False


class EncoderOdomNode(Node):
    """
    Simple differential-drive odometry from 4 quadrature encoder channels.

    Publishes:
      - /wheel_odom  (nav_msgs/Odometry, frame_id=odom, child_frame_id=base_link)
      - /wheel_twist (geometry_msgs/Twist, in base_link frame)
    """

    def __init__(self):
        super().__init__("encoders_node")

        # Parameters (default values from your logs)
        self.declare_parameter("left_a_pin", 12)
        self.declare_parameter("left_b_pin", 16)
        self.declare_parameter("right_a_pin", 7)
        self.declare_parameter("right_b_pin", 11)

        self.declare_parameter("wheel_radius", 0.05)   # m
        self.declare_parameter("wheel_base", 0.24)     # m
        self.declare_parameter("ticks_per_rev", 333.333)

        self.declare_parameter("publish_rate_hz", 30.0)

        # Read parameters
        self.left_a_pin = int(self.get_parameter("left_a_pin").value)
        self.left_b_pin = int(self.get_parameter("left_b_pin").value)
        self.right_a_pin = int(self.get_parameter("right_a_pin").value)
        self.right_b_pin = int(self.get_parameter("right_b_pin").value)

        self.R = float(self.get_parameter("wheel_radius").value)
        self.L = float(self.get_parameter("wheel_base").value)
        self.TPR = float(self.get_parameter("ticks_per_rev").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # State: tick counters
        self.left_ticks = 0
        self.right_ticks = 0
        self._prev_left_ticks = 0
        self._prev_right_ticks = 0

        # Integrated odom pose in odom frame
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Timing
        self.last_time = self.get_clock().now()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "wheel_odom", 10)
        self.twist_pub = self.create_publisher(Twist, "wheel_twist", 10)

        # GPIO setup
        if _HAS_GPIO:
            GPIO.setmode(GPIO.BOARD)

            for pin in (self.left_a_pin, self.left_b_pin,
                        self.right_a_pin, self.right_b_pin):
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            # Quadrature callbacks – both edges
            GPIO.add_event_detect(self.left_a_pin, GPIO.BOTH,
                                  callback=self._left_a_cb, bouncetime=0)
            GPIO.add_event_detect(self.left_b_pin, GPIO.BOTH,
                                  callback=self._left_b_cb, bouncetime=0)
            GPIO.add_event_detect(self.right_a_pin, GPIO.BOTH,
                                  callback=self._right_a_cb, bouncetime=0)
            GPIO.add_event_detect(self.right_b_pin, GPIO.BOTH,
                                  callback=self._right_b_cb, bouncetime=0)

            self.get_logger().info(
                f"Encoders ready. L(A,B)=({self.left_a_pin},{self.left_b_pin}) "
                f"R(A,B)=({self.right_a_pin},{self.right_b_pin}) "
                f"R={self.R:.3f}m L={self.L:.3f}m TPR={self.TPR:.3f}"
            )
        else:
            self.get_logger().warn(
                "Jetson.GPIO not available – encoders will not update!"
            )

        # Timer for integration/publishing
        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self._timer_cb)

    # ----------------- GPIO callbacks -----------------
    def _left_a_cb(self, channel):
        self._update_quadrature(is_left=True)

    def _left_b_cb(self, channel):
        self._update_quadrature(is_left=True)

    def _right_a_cb(self, channel):
        self._update_quadrature(is_left=False)

    def _right_b_cb(self, channel):
        self._update_quadrature(is_left=False)

    def _update_quadrature(self, is_left: bool):
        """Simple x4 quadrature decoding."""
        if not _HAS_GPIO:
            return

        if is_left:
            a = GPIO.input(self.left_a_pin)
            b = GPIO.input(self.left_b_pin)
            if a == b:
                self.left_ticks += 1
            else:
                self.left_ticks -= 1
        else:
            a = GPIO.input(self.right_a_pin)
            b = GPIO.input(self.right_b_pin)
            if a == b:
                self.right_ticks += 1
            else:
                self.right_ticks -= 1

    # ----------------- Timer: integrate + publish -----------------
    def _timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # Copy and reset tick deltas atomically
        left_ticks = self.left_ticks
        right_ticks = self.right_ticks

        d_left_ticks = left_ticks - self._prev_left_ticks
        d_right_ticks = right_ticks - self._prev_right_ticks

        self._prev_left_ticks = left_ticks
        self._prev_right_ticks = right_ticks

        if d_left_ticks == 0 and d_right_ticks == 0:
            # Robot stationary: still publish with zero twist
            self._publish(now, 0.0, 0.0, dt)
            return

        # Convert ticks to distance
        # 2*pi*R per revolution, TPR ticks per revolution
        dist_left = (d_left_ticks / self.TPR) * (2.0 * math.pi * self.R)
        dist_right = (d_right_ticks / self.TPR) * (2.0 * math.pi * self.R)

        # Linear and angular motion
        v = (dist_right + dist_left) / (2.0 * dt)   # m/s
        w = (dist_right - dist_left) / (self.L * dt)  # rad/s

        # Integrate pose in odom frame
        if abs(w) < 1e-6:
            # Approx straight
            dx = v * dt * math.cos(self.yaw)
            dy = v * dt * math.sin(self.yaw)
            dth = 0.0
        else:
            # Exact differential-drive integration
            R_ = v / w
            dth = w * dt
            dx = R_ * (math.sin(self.yaw + dth) - math.sin(self.yaw))
            dy = -R_ * (math.cos(self.yaw + dth) - math.cos(self.yaw))

        self.x += dx
        self.y += dy
        self.yaw = self._normalize_angle(self.yaw + dth)

        self._publish(now, v, w, dt)

    def _publish(self, stamp, v_x, w_z, dt):
        # Odometry message
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Reasonable, *finite* covariances so EKF behaves
        # [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.05   # x
        pose_cov[7] = 0.05   # y
        pose_cov[35] = 0.1   # yaw

        # Make z/roll/pitch "uncertain but finite" (don't use 1e9; EKF can blow up)
        pose_cov[14] = 1.0
        pose_cov[21] = 1.0
        pose_cov[28] = 1.0

        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.1   # vx
        twist_cov[7] = 1.0   # vy (unused)
        twist_cov[35] = 0.2  # wz

        twist_cov[14] = 1.0
        twist_cov[21] = 1.0
        twist_cov[28] = 1.0

        odom.twist.twist.linear.x = float(v_x)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(w_z)
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        # Also publish just the twist in base_link
        tw = Twist()
        tw.linear.x = float(v_x)
        tw.linear.y = 0.0
        tw.linear.z = 0.0
        tw.angular.x = 0.0
        tw.angular.y = 0.0
        tw.angular.z = float(w_z)
        self.twist_pub.publish(tw)

    @staticmethod
    def _normalize_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    finally:
        if _HAS_GPIO:
            GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
