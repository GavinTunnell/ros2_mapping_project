# save as bno055_imu_node.py, then: python3 bno055_imu_node.py
import time
from smbus2 import SMBus
import board, busio
import adafruit_bno055
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

def quat_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return Quaternion(
        x=sr*cp*cy - cr*sp*cy,
        y=cr*sp*sy + sr*cp*sy,
        z=cr*cp*sy - sr*sp*cy,
        w=cr*cp*cy + sr*sp*sy
    )

class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_imu')
        # Try common I²C bus; adjust if needed
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = adafruit_bno055.BNO055_I2C(i2c)  # uses 0x28 by default
        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def tick(self):
        euler = self.bno.euler  # degrees (heading, roll, pitch)
        gyro = self.bno.gyro    # rad/s
        accel = self.bno.linear_acceleration  # m/s^2
        if not euler or not gyro or not accel:
            return
        yaw, roll, pitch = [math.radians(x) for x in euler]  # note ordering
        msg = Imu()
        msg.orientation = quat_from_euler(roll, pitch, yaw)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        self.pub.publish(msg)

def main():
    rclpy.init()
    n = BNO055Node()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
