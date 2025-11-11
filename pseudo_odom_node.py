#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


# ---------- small helpers (no external deps) ----------
def quaternion_from_yaw(yaw: float):
    """Return (x,y,z,w) for Z-rotation only."""
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return (0.0, 0.0, sy, cy)

def yaw_from_quat(x: float, y: float, z: float, w: float):
    """Extract Z yaw from quaternion (XYZ order, ZYX yaw)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PseudoOdom(Node):
    """
    Minimal odom estimator:
      - Yaw from /imu/raw orientation.
      - Forward speed from LiDAR by tracking the range straight ahead.
      - Integrates x,y over time: x += v*dt*cos(yaw), y += v*dt*sin(yaw).
    This is a crude stand-in for encoders, just to keep SLAM updating.
    """

    def __init__(self):
        super().__init__('pseudo_odom_node')

        # Params
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/raw')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('speed_alpha', 0.6)      # low-pass on v
        self.declare_parameter('max_abs_speed', 1.5)    # m/s clamp
        self.declare_parameter('center_avg_beams', 5)   # odd number preferred

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.speed_alpha = float(self.get_parameter('speed_alpha').value)
        self.max_abs_speed = float(self.get_parameter('max_abs_speed').value)
        self.center_avg_beams = int(self.get_parameter('center_avg_beams').value)

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.last_r_center = None
        self.last_scan_time = None
        self.have_imu = False

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # IO
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.on_imu, sensor_qos)
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, sensor_qos)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        self.br = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        # Publish at fixed rate to keep consumers alive even if scans slow down
        self.timer = self.create_timer(0.02, self.on_tick)  # 50 Hz tick

        self.get_logger().info(
            f'Listening: scan={self.scan_topic}, imu={self.imu_topic}; frames: {self.odom_frame}->{self.base_frame}'
        )

    # ----- callbacks -----
    def on_imu(self, msg: Imu):
        # Use IMU orientation for yaw
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        self.yaw = yaw_from_quat(qx, qy, qz, qw)
        self.have_imu = True

    def on_scan(self, msg: LaserScan):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        dt = None if self.last_scan_time is None else (t - self.last_scan_time)
        self.last_scan_time = t

        # pick a small window of beams around "straight ahead"
        # Assume laser frame X-forward at angle 0; if your driver uses different zero, adjust here.
        n = len(msg.ranges)
        if n == 0:
            return

        # beam index at angle closest to 0
        zero_idx = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        half = max(0, (self.center_avg_beams - 1) // 2)
        i0 = max(0, zero_idx - half)
        i1 = min(n - 1, zero_idx + half)
        window = [r for r in msg.ranges[i0:i1 + 1] if math.isfinite(r) and msg.range_min < r < msg.range_max]

        if not window:
            return

        r_center = sum(window) / float(len(window))

        # Estimate forward speed from change in distance to the target straight ahead
        # v ≈ -(r_now - r_prev)/dt. Negative sign: if distance to wall decreases, we move forward.
        if self.last_r_center is not None and dt and dt > 1e-3:
            raw_v = -(r_center - self.last_r_center) / dt
            # Low-pass filter and clamp
            v = self.speed_alpha * raw_v + (1.0 - self.speed_alpha) * self.vx
            v = max(-self.max_abs_speed, min(self.max_abs_speed, v))
            self.vx = v

        self.last_r_center = r_center

    def on_tick(self):
        # integrate pose with the latest v and yaw; if no IMU yet, keep yaw=0
        now = self.get_clock().now().nanoseconds * 1e-9
        if not hasattr(self, '_last_tick'):
            self._last_tick = now
            return
        dt = now - self._last_tick
        self._last_tick = now
        if dt <= 0.0 or dt > 1.0:
            return  # skip unreasonable dt

        # dead-reckon
        self.x += self.vx * dt * math.cos(self.yaw)
        self.y += self.vx * dt * math.sin(self.yaw)

        # publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # simple covariances
        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.10
        ]
        odom.twist.twist.linear.x = float(self.vx)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.pub_odom.publish(odom)

        # publish TF odom->base_link
        if self.br:
            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = odom.pose.pose.position.x
            tf.transform.translation.y = odom.pose.pose.position.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.br.sendTransform(tf)


def main():
    rclpy.init()
    node = PseudoOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
