#!/usr/bin/env python3
import time, struct, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from smbus2 import SMBus, i2c_msg

# BNO055 registers (page 0)
CHIP_ID        = 0x00  # expect 0xA0
PAGE_ID        = 0x07
OPR_MODE       = 0x3D
PWR_MODE       = 0x3E
SYS_TRIGGER    = 0x3F
UNIT_SEL       = 0x3B

# data registers
GYR_DATA_X_LSB = 0x14
EUL_HEADING_LSB= 0x1A
QUAT_W_LSB     = 0x20
LIN_ACC_X_LSB  = 0x28
TEMP           = 0x34
CALIB_STAT     = 0x35

# modes
MODE_CONFIG = 0x00
MODE_IMU    = 0x08  # accel+gyro fusion
MODE_NDOF   = 0x0C  # full fusion

# scales (datasheet)
LSB_PER_DPS       = 16.0       # gyro raw -> deg/s
LSB_PER_MS2       = 100.0      # lin acc raw -> m/s^2
LSB_PER_EULER_DEG = 16.0       # euler raw -> deg
LSB_PER_QUAT      = 16384.0    # quat raw -> unitless

def _s16(u16):
    return struct.unpack('<h', struct.pack('<H', u16))[0]

class BNO055LowLevel:
    def __init__(self, bus:int, addr:int):
        self.busnum = bus
        self.addr = addr
        self.bus = SMBus(bus)

    def write8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def read8(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def read16le(self, reg):
        lo = self.read8(reg)
        hi = self.read8(reg+1)
        return _s16((hi<<8)|lo)

    def read_block(self, reg, length):
        # robust block read using I2C_RDWR
        w = i2c_msg.write(self.addr, [reg])
        r = i2c_msg.read(self.addr, length)
        self.bus.i2c_rdwr(w, r)
        return bytes(r)

    def init(self, mode=MODE_NDOF):
        # wait for chip
        for _ in range(50):
            try:
                if self.read8(CHIP_ID) == 0xA0:
                    break
            except OSError:
                pass
            time.sleep(0.01)
        # to config
        self.write8(OPR_MODE, MODE_CONFIG); time.sleep(0.02)
        # normal power
        self.write8(PWR_MODE, 0x00); time.sleep(0.01)
        # page 0
        self.write8(PAGE_ID, 0x00)
        # units: keep defaults; we convert scaling ourselves
        # start fusion
        self.write8(OPR_MODE, mode); time.sleep(0.02)

    # quaternion (w,x,y,z), unitless
    def read_quat(self):
        b = self.read_block(QUAT_W_LSB, 8)
        w = _s16(b[1]<<8 | b[0]) / LSB_PER_QUAT
        x = _s16(b[3]<<8 | b[2]) / LSB_PER_QUAT
        y = _s16(b[5]<<8 | b[4]) / LSB_PER_QUAT
        z = _s16(b[7]<<8 | b[6]) / LSB_PER_QUAT
        return (w, x, y, z)

    # gyro (rad/s)
    def read_gyro(self):
        b = self.read_block(GYR_DATA_X_LSB, 6)
        gx = _s16(b[1]<<8 | b[0]) / LSB_PER_DPS * (math.pi/180.0)
        gy = _s16(b[3]<<8 | b[2]) / LSB_PER_DPS * (math.pi/180.0)
        gz = _s16(b[5]<<8 | b[4]) / LSB_PER_DPS * (math.pi/180.0)
        return (gx, gy, gz)

    # linear acceleration (m/s^2)
    def read_lin_acc(self):
        b = self.read_block(LIN_ACC_X_LSB, 6)
        ax = _s16(b[1]<<8 | b[0]) / LSB_PER_MS2
        ay = _s16(b[3]<<8 | b[2]) / LSB_PER_MS2
        az = _s16(b[5]<<8 | b[4]) / LSB_PER_MS2
        return (ax, ay, az)

    def alive(self):
        try:
            _ = self.read8(TEMP)
            return True
        except OSError:
            return False

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=20)

        self.declare_parameter('address', 0x28)  # 0x28 = 40
        self.declare_parameter('bus', 1)         # /dev/i2c-1 by default

        self.addr = int(self.get_parameter('address').value)
        self.busnum = int(self.get_parameter('bus').value)

        self.pub_imu = self.create_publisher(Imu, '/imu/raw', qos)
        self.pub_vel = self.create_publisher(Vector3, '/imu/velocity', qos)

        self.bno = BNO055LowLevel(self.busnum, self.addr)
        self.bno.init(MODE_NDOF)
        self.get_logger().info(f'BNO055 init on /dev/i2c-{self.busnum}, addr 0x{self.addr:02x}')

        self.prev_t = time.time()
        self.velocity = [0.0, 0.0, 0.0]
        self.prev_accel = [0.0, 0.0, 0.0]
        self.alpha = 0.8

        self.timer = self.create_timer(0.01, self.loop)  # 100 Hz

    def loop(self):
        if not self.bno.alive():
            return

        t = time.time()
        dt = t - self.prev_t
        if dt <= 0.0:
            return
        self.prev_t = t

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # orientation from quaternion
        try:
            w,x,y,z = self.bno.read_quat()
            msg.orientation.w = float(w)
            msg.orientation.x = float(x)
            msg.orientation.y = float(y)
            msg.orientation.z = float(z)
            msg.orientation_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02,
            ]
        except OSError:
            pass

        # angular velocity
        try:
            gx,gy,gz = self.bno.read_gyro()
            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)
            msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02,
            ]
        except OSError:
            pass

        # linear accel and integrate velocity
        try:
            ax,ay,az = self.bno.read_lin_acc()
            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)
            msg.linear_acceleration_covariance = [
                0.5, 0.0, 0.0,
                0.0, 0.5, 0.0,
                0.0, 0.0, 0.5,
            ]

            fa = [
                self.alpha*ax + (1.0-self.alpha)*self.prev_accel[0],
                self.alpha*ay + (1.0-self.alpha)*self.prev_accel[1],
                self.alpha*az + (1.0-self.alpha)*self.prev_accel[2],
            ]
            self.prev_accel = fa
            for i in range(3):
                self.velocity[i] += fa[i]*dt
                if abs(self.velocity[i]) > 10.0:
                    self.velocity[i] = 0.0

            v = Vector3()
            v.x, v.y, v.z = map(float, self.velocity)
            self.pub_vel.publish(v)
        except OSError:
            pass

        self.pub_imu.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
