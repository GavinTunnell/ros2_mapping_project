#!/usr/bin/env python3
import time, smbus2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

REG_PAGE_ID=0x07; REG_OPR_MODE=0x3D; REG_PWR_MODE=0x3E; REG_UNIT_SEL=0x3B
MODE_CONFIG=0x00; MODE_NDOF=0x0C; PWR_NORMAL=0x00
REG_QUAT_W_LSB=0x20; QUAT_SCALE=1.0/(1<<14)

class BNO055(Node):
    def __init__(self):
        super().__init__('bno055_raw_imu')
        self.declare_parameter('bus', 7)
        self.declare_parameter('address', 0x28)
        self.declare_parameter('frame_id', 'imu_link')
        self.bus_num = int(self.get_parameter('bus').value)
        self.addr    = int(self.get_parameter('address').value)
        self.frame_id= str(self.get_parameter('frame_id').value)
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.bus = smbus2.SMBus(self.bus_num)
        self._init_bno055()
        self.timer = self.create_timer(0.02, self._tick)

    def _w8(self, r, v): self.bus.write_byte_data(self.addr, r, v)
    def _rn(self, r, n): return self.bus.read_i2c_block_data(self.addr, r, n)

    def _init_bno055(self):
        self._w8(REG_PAGE_ID, 0x00)
        self._w8(REG_OPR_MODE, MODE_CONFIG); time.sleep(0.02)
        self._w8(REG_PWR_MODE, PWR_NORMAL);  time.sleep(0.01)
        self._w8(REG_UNIT_SEL, 0x00);        time.sleep(0.01)
        self._w8(REG_OPR_MODE, MODE_NDOF);   time.sleep(0.05)

    def _tick(self):
        try:
            raw = self._rn(REG_QUAT_W_LSB, 8)
            def s16(lo,hi): v=(hi<<8)|lo; return v-0x10000 if v&0x8000 else v
            w=s16(raw[0],raw[1])*QUAT_SCALE; x=s16(raw[2],raw[3])*QUAT_SCALE
            y=s16(raw[4],raw[5])*QUAT_SCALE; z=s16(raw[6],raw[7])*QUAT_SCALE
            m=Imu(); m.header.stamp=self.get_clock().now().to_msg(); m.header.frame_id=self.frame_id
            m.orientation.w=float(w); m.orientation.x=float(x); m.orientation.y=float(y); m.orientation.z=float(z)
            m.orientation_covariance=[0.02,0.0,0.0, 0.0,0.02,0.0, 0.0,0.0,0.02]
            m.angular_velocity_covariance=[-1.0,0.0,0.0, 0.0,-1.0,0.0, 0.0,0.0,-1.0]
            m.linear_acceleration_covariance=[-1.0,0.0,0.0, 0.0,-1.0,0.0, 0.0,0.0,-1.0]
            self.pub.publish(m)
        except Exception as e:
            self.get_logger().warn(f'I2C read failed: {e}')

def main():
    rclpy.init(); n=BNO055(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
