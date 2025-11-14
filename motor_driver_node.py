#!/usr/bin/env python3
#!/usr/bin/env python3
"""ROS 2 motor driver for Jetson GPIO-controlled dual H-bridge."""

from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:  # noqa: SIM105 - keep explicit try/except for clarity when running on dev PCs
    import Jetson.GPIO as GPIO
    HAS_GPIO = True
except Exception:  # pragma: no cover - dev container won't have GPIO
    GPIO = None
    HAS_GPIO = False


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


class _SideDriver:
    """Helper to manage one side of the H-bridge."""

    def __init__(self, enable_pin: int, in_fwd: int, in_rev: int,
                 pwm_freq_hz: float, invert: bool, brake_on_zero: bool):
        self.enable_pin = enable_pin
        self.in_fwd = in_fwd
        self.in_rev = in_rev
        self.brake_on_zero = brake_on_zero
        self.invert = invert
        self.pwm = None

        if HAS_GPIO:
            GPIO.setup(self.enable_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.in_fwd, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.in_rev, GPIO.OUT, initial=GPIO.LOW)
            self.pwm = GPIO.PWM(self.enable_pin, pwm_freq_hz)
            self.pwm.start(0.0)

    def drive(self, duty: float, forward: bool):
        if not HAS_GPIO:
            return

        duty = _clamp(duty, 0.0, 100.0)
        if self.invert:
            forward = not forward
        if duty <= 0.0:
            if self.brake_on_zero:
                GPIO.output(self.in_fwd, GPIO.HIGH)
                GPIO.output(self.in_rev, GPIO.HIGH)
            else:
                GPIO.output(self.in_fwd, GPIO.LOW)
                GPIO.output(self.in_rev, GPIO.LOW)
            self.pwm.ChangeDutyCycle(0.0)
            return

        if forward:
            GPIO.output(self.in_fwd, GPIO.HIGH)
            GPIO.output(self.in_rev, GPIO.LOW)
        else:
            GPIO.output(self.in_fwd, GPIO.LOW)
            GPIO.output(self.in_rev, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(duty)

    def stop(self):
        if not HAS_GPIO:
            return
        GPIO.output(self.in_fwd, GPIO.LOW)
        GPIO.output(self.in_rev, GPIO.LOW)
        if self.pwm:
            self.pwm.ChangeDutyCycle(0.0)
            self.pwm.stop()


class MotorDriverNode(Node):
    """Translate /cmd_vel into differential wheel PWM for Jetson H-bridge."""

    def __init__(self) -> None:
        super().__init__('motor_driver_node')

        # Parameters
        self.declare_parameter('cmd_topic', '/cmd_vel_out')
        self.declare_parameter('wheel_base', 0.24)  # meters
        self.declare_parameter('max_speed_mps', 0.6)
        self.declare_parameter('deadband_mps', 0.03)
        self.declare_parameter('min_duty_pct', 20.0)
        self.declare_parameter('pwm_freq_hz', 500.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('brake_on_stop', True)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        self.declare_parameter('ena', 15)
        self.declare_parameter('enb', 32)
        self.declare_parameter('in1', 23)
        self.declare_parameter('in2', 19)
        self.declare_parameter('in3', 21)
        self.declare_parameter('in4', 18)

        cmd_topic_param = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.cmd_topic = cmd_topic_param or '/cmd_vel_out'
        self.max_speed = float(self.get_parameter('max_speed_mps').value)
        self.deadband = float(self.get_parameter('deadband_mps').value)
        self.min_duty = float(self.get_parameter('min_duty_pct').value) / 100.0
        self.pwm_freq = float(self.get_parameter('pwm_freq_hz').value)
        self.brake_on_zero = bool(self.get_parameter('brake_on_stop').value)
        self.timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        # Pin mapping
        ena = int(self.get_parameter('ena').value)
        enb = int(self.get_parameter('enb').value)
        in1 = int(self.get_parameter('in1').value)
        in2 = int(self.get_parameter('in2').value)
        in3 = int(self.get_parameter('in3').value)
        in4 = int(self.get_parameter('in4').value)

        if not HAS_GPIO:
            self.get_logger().warn('Jetson.GPIO not available; running motor driver in dry-run mode.')
        else:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)

        self.left_driver = _SideDriver(enable_pin=ena, in_fwd=in3, in_rev=in4,
                                       pwm_freq_hz=self.pwm_freq,
                                       invert=bool(self.get_parameter('invert_left').value),
                                       brake_on_zero=self.brake_on_zero)
        self.right_driver = _SideDriver(enable_pin=enb, in_fwd=in1, in_rev=in2,
                                        pwm_freq_hz=self.pwm_freq,
                                        invert=bool(self.get_parameter('invert_right').value),
                                        brake_on_zero=self.brake_on_zero)

        self.last_cmd_time = self.get_clock().now()
        self.current_left = 0.0
        self.current_right = 0.0

        self.sub = self.create_subscription(Twist, self.cmd_topic, self._cmd_cb, 10)
        self.create_timer(0.05, self._watchdog)
        self.get_logger().info(
            f'GPIO motor driver listening on {self.cmd_topic} (max_speed={self.max_speed:.2f} m/s, '
            f'wheel_base={self.wheel_base:.2f} m)'
        )

    # ---- Command processing ----
    def _cmd_cb(self, msg: Twist) -> None:
        self.last_cmd_time = self.get_clock().now()
        left_mps, right_mps = self._twist_to_wheels(msg.linear.x, msg.angular.z)
        self.current_left = left_mps
        self.current_right = right_mps
        self._apply_wheels(left_mps, right_mps)

    def _twist_to_wheels(self, vx: float, wz: float) -> Tuple[float, float]:
        half_base = self.wheel_base / 2.0
        left = vx - wz * half_base
        right = vx + wz * half_base
        return left, right

    def _speed_to_pwm(self, speed: float) -> Tuple[float, bool]:
        forward = speed >= 0.0
        magnitude = abs(speed)
        if magnitude < self.deadband:
            return 0.0, forward
        norm = _clamp(magnitude / self.max_speed, 0.0, 1.0)
        duty = self.min_duty + (1.0 - self.min_duty) * norm
        return duty * 100.0, forward

    def _apply_wheels(self, left_mps: float, right_mps: float) -> None:
        left_duty, left_forward = self._speed_to_pwm(left_mps)
        right_duty, right_forward = self._speed_to_pwm(right_mps)
        self.left_driver.drive(left_duty, left_forward)
        self.right_driver.drive(right_duty, right_forward)

    def _watchdog(self) -> None:
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout_sec:
            if self.current_left != 0.0 or self.current_right != 0.0:
                self.get_logger().warn('cmd_vel timeout; stopping motors')
            self.current_left = 0.0
            self.current_right = 0.0
            self.left_driver.drive(0.0, True)
            self.right_driver.drive(0.0, True)

    def destroy_node(self) -> bool:
        self.left_driver.stop()
        self.right_driver.stop()
        if HAS_GPIO:
            GPIO.cleanup()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
