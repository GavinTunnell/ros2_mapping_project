import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class ImuTwistOdomNode(Node):
    def __init__(self):
        super().__init__('imu_twist_odom_node')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        self.pub = self.create_publisher(Odometry, '/imu_odom', qos)
        self.latest_vel = Vector3()
        self.latest_imu = Imu()
        self.sub_v = self.create_subscription(Vector3, '/imu/velocity', self._vel_cb, qos)
        self.sub_i = self.create_subscription(Imu, '/imu/data', self._imu_cb, qos)
        self.timer = self.create_timer(0.02, self._publish)  # 50 Hz

    def _vel_cb(self, msg: Vector3):
        self.latest_vel = msg

    def _imu_cb(self, msg: Imu):
        self.latest_imu = msg

    def _publish(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        # Use IMU linear velocity in robot frame (assumes imu_link aligned to base_link)
        odom.twist.twist.linear.x = float(self.latest_vel.x)
        odom.twist.twist.linear.y = float(self.latest_vel.y)
        odom.twist.twist.angular.z = float(self.latest_imu.angular_velocity.z)
        # Covariances: high uncertainty since this is integrated accel
        odom.twist.covariance = [
            0.5,0,0, 0,0,0,
            0,1.0,0, 0,0,0,
            0,0,5.0, 0,0,0,
            0,0,0, 2.0,0,0,
            0,0,0, 0,2.0,0,
            0,0,0, 0,0,0.5
        ]
        self.pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTwistOdomNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
