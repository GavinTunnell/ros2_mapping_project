#!/usr/bin/env python3
import math, time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String, Int32

# Jetson GPIO may not exist on dev machines; fail gracefully
try:
    import Jetson.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False


class EncodersNode(Node):
    """
    Robust quadrature encoder reader for two wheels (Rover 5).
    - Handles active_low polarity, illegal transitions, per-pin deglitch.
    - Optional polling fallback.
    - Publishes /wheel_odom (twist only), plus debug topics:
      /left_wheel/velocity (Float32, m/s)
      /right_wheel/velocity
      /left_wheel/direction (String: FWD/REV/STOP)
      /right_wheel/direction
      /left_wheel/ticks (Int32, ticks since last publish)
      /right_wheel/ticks
    """

    def __init__(self):
        super().__init__('encoders_node')

        # --- Pins (BOARD numbering) ---
        self.declare_parameter('left_A', 12)
        self.declare_parameter('left_B', 16)
        self.declare_parameter('right_A', 7)
        self.declare_parameter('right_B', 11)

        # --- Geometry & scaling ---
        self.declare_parameter('wheel_radius', 0.05)        # meters
        self.declare_parameter('wheel_base',   0.24)        # meters
        self.declare_parameter('ticks_per_rev', 333.3333)   # ticks per wheel rev (Rover 5 ≈ 1000/3 @4x)

        # --- Timing / edge handling ---
        self.declare_parameter('publish_rate_hz', 10.0)     # odom/debug rate
        self.declare_parameter('debounce_ms', 0)            # Jetson ignores pull-up config; this is ISR bouncetime
        self.declare_parameter('edge_min_us', 200)          # per-pin minimum microseconds between edges (deglitch)
        self.declare_parameter('poll_rate_hz', 0.0)         # 0 = use interrupts; >0 = poll at this rate

        # --- Polarity & channel selection ---
        self.declare_parameter('left_active_low', True)
        self.declare_parameter('right_active_low', True)
        self.declare_parameter('left_only_A_edges', False)  # True = count A only (debug)
        self.declare_parameter('right_only_A_edges', False)

        # --- Direction smoothing ---
        self.declare_parameter('dir_hold_ms', 300)          # min time to keep a direction before flipping
        self.declare_parameter('dir_min_ticks', 4)          # require this many net ticks to flip
        self.declare_parameter('vel_alpha', 0.4)            # EMA smoothing for per-wheel velocity
        self.declare_parameter('vel_eps', 0.01)             # m/s deadband for STOP label

        # Params -> members
        self.lA = int(self.get_parameter('left_A').value)
        self.lB = int(self.get_parameter('left_B').value)
        self.rA = int(self.get_parameter('right_A').value)
        self.rB = int(self.get_parameter('right_B').value)
        self.r  = float(self.get_parameter('wheel_radius').value)
        self.L  = float(self.get_parameter('wheel_base').value)
        self.tpr = float(self.get_parameter('ticks_per_rev').value)

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.debounce_ms = int(self.get_parameter('debounce_ms').value)
        self.edge_min_us = int(self.get_parameter('edge_min_us').value)
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)

        self.left_active_low  = bool(self.get_parameter('left_active_low').value)
        self.right_active_low = bool(self.get_parameter('right_active_low').value)
        self.left_only_A_edges  = bool(self.get_parameter('left_only_A_edges').value)
        self.right_only_A_edges = bool(self.get_parameter('right_only_A_edges').value)

        self._dir_hold_ms   = int(self.get_parameter('dir_hold_ms').value)
        self._dir_min_ticks = int(self.get_parameter('dir_min_ticks').value)
        self._vel_alpha     = float(self.get_parameter('vel_alpha').value)
        self._vel_eps       = float(self.get_parameter('vel_eps').value)

        # QoS & pubs
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', qos)

        # Debug publishers
        self.l_vel_pub = self.create_publisher(Float32, '/left_wheel/velocity', 10)
        self.r_vel_pub = self.create_publisher(Float32, '/right_wheel/velocity', 10)
        self.l_dir_pub = self.create_publisher(String,  '/left_wheel/direction', 10)
        self.r_dir_pub = self.create_publisher(String,  '/right_wheel/direction', 10)
        self.l_tick_pub= self.create_publisher(Int32,   '/left_wheel/ticks', 10)
        self.r_tick_pub= self.create_publisher(Int32,   '/right_wheel/ticks', 10)

        # State
        self._count_lock = threading.Lock()
        self._lticks = 0
        self._rticks = 0
        self._prev_time = time.time()

        # Last states (2-bit) and last-edge timestamps (us) per pin
        self._l_prev = 0
        self._r_prev = 0
        self._last_edge_us = {self.lA: 0, self.lB: 0, self.rA: 0, self.rB: 0}

        # Velocity EMA state
        self._l_v_f = 0.0
        self._r_v_f = 0.0

        # Direction latch state
        self._l_dir = "STOP"
        self._r_dir = "STOP"
        self._l_dir_last_change = time.time()
        self._r_dir_last_change = time.time()
        self._l_tick_accum = 0
        self._r_tick_accum = 0

        # Precompute distance per tick
        self._dist_per_tick = (2.0 * math.pi * self.r) / self.tpr

        if HAS_GPIO:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            for p in (self.lA, self.lB, self.rA, self.rB):
                GPIO.setup(p, GPIO.IN)  # Jetson ignores pull_up_down; use external pull-ups

            self._l_prev = self._read_pair(self.lA, self.lB, left=True)
            self._r_prev = self._read_pair(self.rA, self.rB, left=False)

            if self.poll_rate_hz > 0.0:
                # Polling fallback
                self._poll_timer = self.create_timer(1.0 / self.poll_rate_hz, self._poll_once)
            else:
                # Interrupts
                kwargs = {}
                if self.debounce_ms > 0:
                    kwargs['bouncetime'] = self.debounce_ms
                # Left
                GPIO.add_event_detect(self.lA, GPIO.BOTH, callback=self._on_left_edge, **kwargs)
                if not self.left_only_A_edges:
                    GPIO.add_event_detect(self.lB, GPIO.BOTH, callback=self._on_left_edge, **kwargs)
                # Right
                GPIO.add_event_detect(self.rA, GPIO.BOTH, callback=self._on_right_edge, **kwargs)
                if not self.right_only_A_edges:
                    GPIO.add_event_detect(self.rB, GPIO.BOTH, callback=self._on_right_edge, **kwargs)
        else:
            self.get_logger().warn('Jetson.GPIO not available; encoder counts will stay zero.')

        # Publish timer
        self.create_timer(max(0.01, 1.0 / rate_hz), self._publish)

        self.get_logger().info(
            f'Encoders ready. L(A,B)=({self.lA},{self.lB}) R(A,B)=({self.rA},{self.rB}) '
            f'R={self.r:.3f}m L={self.L:.3f}m TPR={self.tpr:.3f} '
            f'active_low(L/R)={self.left_active_low}/{self.right_active_low} '
            f'poll_rate_hz={self.poll_rate_hz}'
        )

    # --- Utilities ---
    def _now_us(self) -> int:
        return int(time.time() * 1e6)

    def _read_bit(self, pin: int, invert: bool) -> int:
        v = GPIO.input(pin)
        if invert:
            v = 0 if v else 1
        return 1 if v else 0

    def _read_pair(self, A: int, B: int, left: bool) -> int:
        inv = self.left_active_low if left else self.right_active_low
        a = self._read_bit(A, inv)
        b = self._read_bit(B, inv)
        return (a << 1) | b

    # --- Edge handlers (interrupt mode) ---
    def _edge_allowed(self, pin: int) -> bool:
        if self.edge_min_us <= 0:
            return True
        t = self._now_us()
        if t - self._last_edge_us[pin] < self.edge_min_us:
            return False
        self._last_edge_us[pin] = t
        return True

    def _apply_transition(self, prev: int, curr: int, left: bool):
        # Ignore repeats
        if curr == prev:
            return 0
        # Illegal transition: both bits changed (00->11 or 11->00)
        if (prev ^ curr) == 0b11:
            return 0

        # Valid single-bit transition
        # Order of states around the quadrature wheel: 00 -> 01 -> 11 -> 10 -> 00 (FWD)
        forward_steps = { (0b00,0b01), (0b01,0b11), (0b11,0b10), (0b10,0b00) }
        backward_steps= { (b,a) for (a,b) in forward_steps }

        step = 0
        if (prev, curr) in forward_steps:
            step = +1
        elif (prev, curr) in backward_steps:
            step = -1

        if step != 0:
            with self._count_lock:
                if left: self._lticks += step
                else:    self._rticks += step
        return step

    def _on_left_edge(self, ch):
        if not HAS_GPIO or not self._edge_allowed(ch):
            return
        curr = self._read_pair(self.lA, self.lB, left=True)
        prev = self._l_prev
        self._apply_transition(prev, curr, left=True)
        self._l_prev = curr

    def _on_right_edge(self, ch):
        if not HAS_GPIO or not self._edge_allowed(ch):
            return
        curr = self._read_pair(self.rA, self.rB, left=False)
        prev = self._r_prev
        self._apply_transition(prev, curr, left=False)
        self._r_prev = curr

    # --- Polling mode (optional) ---
    def _poll_once(self):
        if not HAS_GPIO:
            return
        # Left
        curr = self._read_pair(self.lA, self.lB, left=True)
        prev = self._l_prev
        self._apply_transition(prev, curr, left=True)
        self._l_prev = curr
        # Right
        curr = self._read_pair(self.rA, self.rB, left=False)
        prev = self._r_prev
        self._apply_transition(prev, curr, left=False)
        self._r_prev = curr

    # --- Direction latch update ---
    def _update_dir(self, current_dir: str, last_change_ts: float, tick_accum: int, v_f: float):
        # Treat tiny speeds as STOP
        if abs(v_f) < self._vel_eps:
            return "STOP", last_change_ts, tick_accum

        now_ts = time.time()
        hold_ok = (now_ts - last_change_ts) * 1000.0 >= self._dir_hold_ms

        if tick_accum >= self._dir_min_ticks and hold_ok:
            return "FWD", now_ts, 0
        if tick_accum <= -self._dir_min_ticks and hold_ok:
            return "REV", now_ts, 0

        # bleed accumulator slowly to avoid unbounded growth
        if tick_accum > 0:
            tick_accum = max(0, tick_accum - 1)
        elif tick_accum < 0:
            tick_accum = min(0, tick_accum + 1)
        return current_dir, last_change_ts, tick_accum

    # --- Publish odom + debug ---
    def _publish(self):
        now = time.time()
        dt = now - self._prev_time
        if dt <= 0.0:
            return
        self._prev_time = now

        with self._count_lock:
            lt = self._lticks
            rt = self._rticks
            self._lticks = 0
            self._rticks = 0

        # accumulate for direction decision
        self._l_tick_accum += lt
        self._r_tick_accum += rt

        # instantaneous velocities
        l_v = (lt * self._dist_per_tick) / dt
        r_v = (rt * self._dist_per_tick) / dt

        # EMA smoothing
        a = self._vel_alpha
        self._l_v_f = (1 - a) * self._l_v_f + a * l_v
        self._r_v_f = (1 - a) * self._r_v_f + a * r_v

        # Update latched directions
        self._l_dir, self._l_dir_last_change, self._l_tick_accum = self._update_dir(
            self._l_dir, self._l_dir_last_change, self._l_tick_accum, self._l_v_f
        )
        self._r_dir, self._r_dir_last_change, self._r_tick_accum = self._update_dir(
            self._r_dir, self._r_dir_last_change, self._r_tick_accum, self._r_v_f
        )

        # Body twist (for EKF)
        v = 0.5 * (self._l_v_f + self._r_v_f)
        w = (self._r_v_f - self._l_v_f) / self.L if self.L != 0.0 else 0.0

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x  = float(v)
        odom.twist.twist.angular.z = float(w)
        odom.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]
        self.odom_pub.publish(odom)

        # Debug topics (filtered velocities + latched dirs + raw tick burst)
        self.l_vel_pub.publish(Float32(data=float(self._l_v_f)))
        self.r_vel_pub.publish(Float32(data=float(self._r_v_f)))
        self.l_tick_pub.publish(Int32(data=int(lt)))
        self.r_tick_pub.publish(Int32(data=int(rt)))
        self.l_dir_pub.publish(String(data=self._l_dir))
        self.r_dir_pub.publish(String(data=self._r_dir))

    def destroy_node(self):
        try:
            if HAS_GPIO:
                GPIO.cleanup()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncodersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
