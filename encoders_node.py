#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node

try:
    import Jetson.GPIO as GPIO
    _HAS_GPIO = True
except Exception:
    _HAS_GPIO = False

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * yaw
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class EncoderNode(Node):
    """
    Differential-drive wheel odometry node.

    Publishes:
      - /wheel_odom (Odometry)
      - /wheel_twist (TwistWithCovarianceStamped)

    No special "ignore vx while turning" hacks here.
    """

    def __init__(self):
        super().__init__('encoders_node')

        # --- Parameters ---
        self.declare_parameter('left_a_pin', 12)
        self.declare_parameter('left_b_pin', 16)
        self.declare_parameter('right_a_pin', 7)
        self.declare_parameter('right_b_pin', 11)

        self.declare_parameter('wheel_radius', 0.05)   # m
        self.declare_parameter('wheel_base', 0.24)     # m
        self.declare_parameter('ticks_per_rev', 333.3333)
        self.declare_parameter('publish_rate_hz', 20.0)

        # --- Read params ---
        self.left_a_pin = int(self.get_parameter('left_a_pin').value)
        self.left_b_pin = int(self.get_parameter('left_b_pin').value)
        self.right_a_pin = int(self.get_parameter('right_a_pin').value)
        self.right_b_pin = int(self.get_parameter('right_b_pin').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        # --- State ---
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()
        self.lock = threading.Lock()

        # --- Publishers + TF ---
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.twist_pub = self.create_publisher(TwistWithCovarianceStamped, 'wheel_twist', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- GPIO setup ---
        if _HAS_GPIO:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            GPIO.setup(self.left_a_pin, GPIO.IN)
            GPIO.setup(self.left_b_pin, GPIO.IN)
            GPIO.setup(self.right_a_pin, GPIO.IN)
            GPIO.setup(self.right_b_pin, GPIO.IN)

            GPIO.add_event_detect(self.left_a_pin, GPIO.BOTH, callback=self._left_callback)
            GPIO.add_event_detect(self.right_a_pin, GPIO.BOTH, callback=self._right_callback)
        else:
            self.get_logger().warn("Jetson.GPIO not available; encoder pins won’t actually count.")

        self.get_logger().info(
            f"Encoders ready. L(A,B)=({self.left_a_pin},{self.left_b_pin}) "
            f"R(A,B)=({self.right_a_pin},{self.right_b_pin}) "
            f"R={self.wheel_radius:.3f}m L={self.wheel_base:.3f}m TPR={self.ticks_per_rev:.3f}"
        )

        # --- Timer ---
        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self._timer_cb)

    # ------------------ encoder callbacks ------------------

    def _left_callback(self, channel: int):
        if not _HAS_GPIO:
            return
        a = GPIO.input(self.left_a_pin)
        b = GPIO.input(self.left_b_pin)
        with self.lock:
            if a == b:
                self.left_ticks += 1
            else:
                self.left_ticks -= 1

    def _right_callback(self, channel: int):
        if not _HAS_GPIO:
            return
        a = GPIO.input(self.right_a_pin)
        b = GPIO.input(self.right_b_pin)
        with self.lock:
            if a == b:
                self.right_ticks += 1
            else:
                self.right_ticks -= 1

    # ------------------ main timer ------------------

    def _timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        with self.lock:
            lt = self.left_ticks
            rt = self.right_ticks

        dlt = lt - self.prev_left_ticks
        drt = rt - self.prev_right_ticks

        self.prev_left_ticks = lt
        self.prev_right_ticks = rt

        # If no new ticks, just republish pose at same spot
        if dlt == 0 and drt == 0:
            self._publish(now, 0.0, 0.0)
            return

        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        dl = dlt * meters_per_tick
        dr = drt * meters_per_tick

        vl = dl / dt
        vr = dr / dt

        v = 0.5 * (vl + vr)
        omega_enc = (vr - vl) / self.wheel_base

        # Integrate pose normally
        if abs(omega_enc) < 1e-6:
            dx = v * dt * math.cos(self.yaw)
            dy = v * dt * math.sin(self.yaw)
            dtheta = 0.0
        else:
            R = v / omega_enc
            dtheta = omega_enc * dt
            dx = R * (math.sin(self.yaw + dtheta) - math.sin(self.yaw))
            dy = -R * (math.cos(self.yaw + dtheta) - math.cos(self.yaw))

        self.x += dx
        self.y += dy
        self.yaw = self._normalize_angle(self.yaw + dtheta)

        self._publish(now, v, omega_enc)

    # ------------------ publish odom/twist/tf ------------------

    def _publish(self, stamp, v: float, omega: float):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        pose_cov = [0.0] * 36
        pose_cov[0] = 0.05    # x
        pose_cov[7] = 0.05    # y
        pose_cov[35] = 0.10   # yaw
        odom.pose.covariance = pose_cov

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.05    # vx
        twist_cov[35] = 0.10   # wz
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        tw = TwistWithCovarianceStamped()
        tw.header.stamp = odom.header.stamp
        tw.header.frame_id = 'base_link'
        tw.twist.twist = odom.twist.twist
        tw.twist.covariance = twist_cov
        self.twist_pub.publish(tw)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        a = math.fmod(angle + math.pi, 2.0 * math.pi)
        if a < 0.0:
            a += 2.0 * math.pi
        return a - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    finally:
        if _HAS_GPIO:
            GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
