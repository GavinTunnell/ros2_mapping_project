#!/usr/bin/env python3
# Dual-PCA9685 motor driver for /cmd_vel (no GPIO; register-level I2C)
#  - PCA @ 0x41: channels [EnA, In1, In2]  (default 0,1,2)
#  - PCA @ 0x60: channels [EnB, In3, In4]  (default 0,1,2)
# Forward logic (default):
#   Right side (EnA/In1/In2): forward => IN1=1, IN2=0
#   Left  side (EnB/In3/In4): forward => IN3=1, IN4=0
# STOP/coast = inputs LOW (both 0). If brake_on_zero=True, both HIGH.
#
# Notes:
#  - Use strings for I2C addresses ("0x41", "0x60") to avoid ROS param type issues
#  - Use `max_ang_cmd` so wz=max_ang_cmd gives full-power counter-rotation
#  - `min_duty_pct` ensures startup torque; typical 30–40% for DC gear motors
#
# Example:
#   source /opt/ros/humble/setup.bash
#   python3 motor_driver_pca_reg_dual.py --ros-args \
#     -p ena_addr:="'0x41'" -p enb_addr:="'0x60'" \
#     -p ena_channel:=0 -p in1_channel:=1 -p in2_channel:=2 \
#     -p enb_channel:=0 -p in3_channel:=1 -p in4_channel:=2 \
#     -p pwm_freq_hz:=1000.0 -p min_duty_pct:=35.0 \
#     -p max_lin:=0.8 -p max_ang_cmd:=1.2 -p deadband:=0.03 \
#     -p invert_right:=true -p invert_left:=false

import math
import time
import smbus
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ---------- PCA9685 low-level ----------
_PCA_MODE1        = 0x00
_PCA_MODE2        = 0x01
_PCA_PRESCALE     = 0xFE
_PCA_LED0_ON_L    = 0x06
# MODE1 bits
_RESTART          = 0x80
_SLEEP            = 0x10
_AI               = 0x20
# MODE2 bits
_OUTDRV           = 0x04  # totem-pole

class PCA9685LowLevel:
    def __init__(self, busnum: int, address: int, freq_hz: float):
        self.bus    = smbus.SMBus(busnum)
        self.addr   = address
        self.freq_hz= float(freq_hz)
        self._init_chip()

    def _write8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def _read8(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def _init_chip(self):
        # reset
        self._write8(_PCA_MODE1, 0x00)  # all call off, AI off initially
        self._write8(_PCA_MODE2, _OUTDRV)
        time.sleep(0.005)
        # sleep to set prescale
        oldmode = self._read8(_PCA_MODE1)
        self._write8(_PCA_MODE1, (oldmode | _SLEEP) & 0xFF)
        prescale = int(round(25_000_000.0 / (4096.0 * self.freq_hz) - 1.0))
        prescale = max(3, min(255, prescale))
        self._write8(_PCA_PRESCALE, prescale)
        # wake, auto-increment
        self._write8(_PCA_MODE1, (oldmode & ~_SLEEP) | _AI)
        time.sleep(0.005)
        # restart
        self._write8(_PCA_MODE1, ((self._read8(_PCA_MODE1) | _RESTART) | _AI) & 0xFF)

    def set_pwm_raw(self, channel: int, on_count: int, off_count: int):
        base = _PCA_LED0_ON_L + 4 * channel
        self._write8(base + 0, on_count & 0xFF)
        self._write8(base + 1, (on_count >> 8) & 0x0F)
        self._write8(base + 2, off_count & 0xFF)
        self._write8(base + 3, (off_count >> 8) & 0x0F)

    def set_pwm_duty(self, channel: int, duty: float):
        # duty in [0.0, 1.0]
        duty = max(0.0, min(1.0, duty))
        if duty <= 0.0:
            # fully off
            self.set_pwm_raw(channel, 0, 0)
        elif duty >= 1.0:
            # fully on (use ON=0x1000 trick or set OFF=0)
            self.set_pwm_raw(channel, 0, 4095)
        else:
            off = int(round(duty * 4095))
            off = max(1, min(4094, off))
            self.set_pwm_raw(channel, 0, off)

    def set_pin_digital(self, channel: int, high: bool):
        self.set_pwm_duty(channel, 1.0 if high else 0.0)

# ---------- Helpers ----------
def _clip(x, lo, hi): return lo if x < lo else hi if x > hi else x

# ---------- ROS Node ----------
class MotorDriverPCADual(Node):
    def __init__(self):
        super().__init__('motor_driver_pca_dual')

        # Addresses as STRINGS so CLI -p works cleanly
        self.declare_parameter('ena_addr', '0x41')
        self.declare_parameter('enb_addr', '0x60')

        # Channel mapping (defaults: En*=0, inputs 1&2)
        self.declare_parameter('ena_channel', 0)
        self.declare_parameter('in1_channel', 1)
        self.declare_parameter('in2_channel', 2)

        self.declare_parameter('enb_channel', 0)
        self.declare_parameter('in3_channel', 1)
        self.declare_parameter('in4_channel', 2)

        # Control params
        self.declare_parameter('pwm_freq_hz', 1000.0)
        self.declare_parameter('min_duty_pct', 35.0)   # %
        self.declare_parameter('deadband', 0.03)       # normalized
        self.declare_parameter('max_lin', 0.8)         # m/s (for normalization)
        self.declare_parameter('max_ang_cmd', 1.2)     # rad/s that maps to full turn
        self.declare_parameter('brake_on_zero', False) # if True => both inputs HIGH on zero

        # Side inversion (flip forward)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('invert_left',  False)

        # Which side EnA drives (True=left)
        self.declare_parameter('map_enA_to_left', True)

        # I2C bus number (Jetson 40-pin I2C is usually bus 1)
        self.declare_parameter('i2c_bus', 1)

        # Read params
        ena_addr_str = self.get_parameter('ena_addr').get_parameter_value().string_value
        enb_addr_str = self.get_parameter('enb_addr').get_parameter_value().string_value
        self.ena_addr = int(ena_addr_str, 16)
        self.enb_addr = int(enb_addr_str, 16)

        self.ena_ch = int(self.get_parameter('ena_channel').value)
        self.in1_ch = int(self.get_parameter('in1_channel').value)
        self.in2_ch = int(self.get_parameter('in2_channel').value)

        self.enb_ch = int(self.get_parameter('enb_channel').value)
        self.in3_ch = int(self.get_parameter('in3_channel').value)
        self.in4_ch = int(self.get_parameter('in4_channel').value)

        self.freq_hz   = float(self.get_parameter('pwm_freq_hz').value)
        self.min_duty  = float(self.get_parameter('min_duty_pct').value) / 100.0
        self.deadband  = float(self.get_parameter('deadband').value)
        self.max_lin   = float(self.get_parameter('max_lin').value)
        self.max_ang   = float(self.get_parameter('max_ang_cmd').value)
        self.brake0    = bool(self.get_parameter('brake_on_zero').value)

        self.inv_r     = bool(self.get_parameter('invert_right').value)
        self.inv_l     = bool(self.get_parameter('invert_left').value)
        self.mapA_left = bool(self.get_parameter('map_enA_to_left').value)

        busnum         = int(self.get_parameter('i2c_bus').value)

        # Init both PCA chips
        self.pcaA = PCA9685LowLevel(busnum, self.ena_addr, self.freq_hz)
        self.pcaB = PCA9685LowLevel(busnum, self.enb_addr, self.freq_hz)

        # Map sides to boards/channels
        if self.mapA_left:
            self.left  = dict(pca=self.pcaA, en=self.ena_ch, fwd=self.in3_ch, rev=self.in4_ch)  # use IN3/IN4 semantics for left
            self.right = dict(pca=self.pcaB, en=self.enb_ch, fwd=self.in1_ch, rev=self.in2_ch)  # use IN1/IN2 semantics for right
        else:
            self.left  = dict(pca=self.pcaB, en=self.enb_ch, fwd=self.in3_ch, rev=self.in4_ch)
            self.right = dict(pca=self.pcaA, en=self.ena_ch, fwd=self.in1_ch, rev=self.in2_ch)

        # Ensure stopped
        self._coast_side(self.left)
        self._coast_side(self.right)

        # Subscriber
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.get_logger().info(
            f"PCA A=0x{self.ena_addr:02X} (EnA={self.ena_ch}, In1={self.in1_ch}, In2={self.in2_ch}) | "
            f"PCA B=0x{self.enb_addr:02X} (EnB={self.enb_ch}, In3={self.in3_ch}, In4={self.in4_ch}) | "
            f"freq={self.freq_hz}Hz min_duty={self.min_duty*100:.0f}%"
        )

    # ----- side helpers -----
    def _apply_side(self, side, forward: bool, duty: float, brake: bool):
        pca = side['pca']; en = side['en']; fwd = side['fwd']; rev = side['rev']
        duty = _clip(duty, 0.0, 1.0)

        if duty <= 0.0:
            # STOP
            pca.set_pwm_duty(en, 0.0)
            if self.brake0 or brake:
                pca.set_pin_digital(fwd, True)
                pca.set_pin_digital(rev, True)
            else:
                pca.set_pin_digital(fwd, False)
                pca.set_pin_digital(rev, False)
            return

        # Direction
        if forward:
            pca.set_pin_digital(fwd, True)
            pca.set_pin_digital(rev, False)
        else:
            pca.set_pin_digital(fwd, False)
            pca.set_pin_digital(rev, True)

        # Enable PWM
        pca.set_pwm_duty(en, duty)

    def _coast_side(self, side):
        side['pca'].set_pwm_duty(side['en'], 0.0)
        side['pca'].set_pin_digital(side['fwd'], False)
        side['pca'].set_pin_digital(side['rev'], False)

    # ----- mixer -----
    def on_cmd(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        # Normalize to [-1,1]
        mix_v = _clip(vx / max(1e-6, self.max_lin), -1.0, 1.0)
        mix_w = _clip(wz / max(1e-6, self.max_ang), -1.0, 1.0)

        r = mix_v + mix_w
        l = mix_v - mix_w

        # Keep within [-1,1] preserving ratio
        peak = max(1.0, abs(r), abs(l))
        r /= peak; l /= peak

        def map_duty(x):
            ax = abs(x)
            if ax < self.deadband:
                return 0.0, True
            return max(self.min_duty, ax), False

        duty_r, off_r = map_duty(r)
        duty_l, off_l = map_duty(l)

        # Apply inversion flags
        dir_r = (r >= 0.0)
        dir_l = (l >= 0.0)
        if self.inv_r: dir_r = not dir_r
        if self.inv_l: dir_l = not dir_l

        # Drive sides
        if off_r:
            self._apply_side(self.right, True, 0.0, brake=False)
        else:
            self._apply_side(self.right, dir_r, duty_r, brake=False)

        if off_l:
            self._apply_side(self.left, True, 0.0, brake=False)
        else:
            self._apply_side(self.left, dir_l, duty_l, brake=False)

    def destroy_node(self):
        try:
            self._coast_side(self.left)
            self._coast_side(self.right)
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = MotorDriverPCADual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
