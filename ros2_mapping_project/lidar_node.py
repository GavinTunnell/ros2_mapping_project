import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import serial
import math

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.publisher = self.create_publisher(LaserScan, '/scan', qos)

        port = self.declare_parameter('port', '/dev/ttyUSB0').get_parameter_value().string_value
        baud = self.declare_parameter('baud', 115200).get_parameter_value().integer_value
        try:
            self.serial = serial.Serial(port, baud, timeout=0.05)
            self.get_logger().info(f'LIDAR connected on {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Cannot open LIDAR serial: {e}')
            self.serial = None

        self.angle_min = -math.pi
        self.angle_max =  math.pi
        self.angle_increment = 0.01
        self.range_min = 0.15
        self.range_max = 12.0
        self.N = int((self.angle_max - self.angle_min)/self.angle_increment) + 1

        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz

    def publish_scan(self):
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = float(self.angle_min)
        scan.angle_max = float(self.angle_max)
        scan.angle_increment = float(self.angle_increment)
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.range_max)
        # Placeholder data; use sllidar_ros2 driver in launch for real scans
        scan.ranges = [float('nan')] * self.N
        self.publisher.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
