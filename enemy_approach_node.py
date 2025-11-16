#!/usr/bin/env python3
"""
enemy_approach_node.py

Behavior summary:
  - SEARCH:
      * Rover stopped.
      * Servo sweeps 30° ↔ 150°.
      * Autofocus sweeps.
  - ALIGN_COUPLED (on first detection):
      * Start at current servo angle.
      * Servo slowly pans toward EXACTLY 90° (servo_center_deg).
      * While servo is moving, rover rotates in the opposite direction
        at a fixed speed (turn_speed_coupled).
      * When servo reaches 90° (within small tol), snap to 90° and go to DRIVE.
  - DRIVE:
      * Servo held at 90°.
      * Rover drives forward (forward_speed ≥ 0.35 m/s) using bbox x-error to
        adjust angular velocity. Turning uses a boost:
          - Turn start/direction change → spike to 0.8 rad/s
          - Then settle to ≥ 0.5 rad/s.
  - ALIGN_SERVO:
      * When bbox area ≥ 40% of frame, rover stops.
      * Servo alone pans slowly to center the target in the image.
  - FIRE:
      * When target centered after ALIGN_SERVO, laser ON for 10 s, rover stopped.
      * Then back to SEARCH.

Assumes:
  - Motor driver listens to /cmd_vel.
  - Servo is on a PCA9685 at I2C addr 0x42, channel 0 (50 Hz).
  - Laser is on Jetson BOARD pin 22 (HIGH = ON, LOW = OFF).
"""

import os
import sys
import time
import cv2
import numpy as np
import smbus

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from ultralytics import YOLO
import Jetson.GPIO as GPIO

# Ensure Focuser.py is importable from your package dir
sys.path.append(os.path.dirname(__file__))
from Focuser import Focuser


# ---------- PCA9685 low-level ----------
_PCA_MODE1        = 0x00
_PCA_MODE2        = 0x01
_PCA_PRESCALE     = 0xFE
_PCA_LED0_ON_L    = 0x06
_RESTART          = 0x80
_SLEEP            = 0x10
_AI               = 0x20
_OUTDRV           = 0x04  # totem-pole


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


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
        self._write8(_PCA_MODE1, 0x00)
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
        duty = max(0.0, min(1.0, duty))
        if duty <= 0.0:
            # fully off
            self.set_pwm_raw(channel, 0, 0)
        elif duty >= 1.0:
            # fully on
            self.set_pwm_raw(channel, 0, 4095)
        else:
            off = int(round(duty * 4095))
            off = max(1, min(4094, off))
            self.set_pwm_raw(channel, 0, off)


def angle_to_duty(angle_deg: float, freq_hz: float = 50.0) -> float:
    """
    Map servo angle [0,180] to duty fraction for PCA9685 at freq_hz.
    Uses 0.5 ms -> 0°, 2.5 ms -> 180°.
    """
    MIN_US = 500
    MAX_US = 2500
    angle = max(0.0, min(180.0, float(angle_deg)))
    us = MIN_US + (MAX_US - MIN_US) * angle / 180.0
    period_us = 1_000_000.0 / freq_hz
    duty = us / period_us  # 0.0..1.0
    return duty


class EnemyApproachAutofocusNode(Node):
    STATE_SEARCH        = 0
    STATE_ALIGN_COUPLED = 1
    STATE_DRIVE         = 2
    STATE_ALIGN_SERVO   = 3
    STATE_FIRE          = 4

    def __init__(self):
        super().__init__('enemy_approach_autofocus')

        # ---- Detection params ----
        self.declare_parameter('engine_path', 'green_specific.engine')
        self.declare_parameter('device', 0)
        self.declare_parameter('imgsz', 704)
        self.declare_parameter('conf_thresh', 0.25)
        self.declare_parameter('half', True)
        self.declare_parameter('show_window', True)
        self.declare_parameter('target_label', 'toy soldier')

        # ---- Rover motion params ----
        self.declare_parameter('forward_speed', 0.35)      # m/s, >= 0.3 so motors move
        self.declare_parameter('max_turn_speed', 0.8)      # rad/s for tracking
        self.declare_parameter('min_turn_speed', 0.5)      # rad/s after boost
        self.declare_parameter('yaw_gain', 1.0)            # P-gain for bbox error
        self.declare_parameter('coverage_thresh', 0.40)    # 40% frame area
        self.declare_parameter('lost_timeout', 2.0)        # seconds lost → SEARCH
        self.declare_parameter('align_tol_n', 0.03)        # |x_n-0.5| tol for DRIVE align

        # Coupled align turn speed (fixed, avoids stall)
        self.declare_parameter('turn_speed_coupled', 0.8)  # rad/s

        # Turn boost behavior
        self.declare_parameter('turn_boost_speed', 0.8)    # spike speed
        self.declare_parameter('turn_boost_duration', 0.15)  # seconds

        # ---- Servo params (PCA9685) ----
        self.declare_parameter('servo_scan_min_deg', 30.0)
        self.declare_parameter('servo_scan_max_deg', 150.0)
        self.declare_parameter('servo_scan_step_deg', 1.0)
        self.declare_parameter('servo_center_deg', 90.0)      # logical straight
        self.declare_parameter('servo_center_step_deg', 1.5)  # deg per loop in ALIGN_COUPLED
        self.declare_parameter('servo_center_tol_deg', 0.2)   # when done, snap to 90

        self.declare_parameter('pca_bus', 1)
        self.declare_parameter('pca_addr', 0x42)
        self.declare_parameter('pca_freq_hz', 50.0)
        self.declare_parameter('servo_channel', 0)

        # ALIGN_SERVO fine-pan params
        self.declare_parameter('align_servo_gain_deg', 30.0)
        self.declare_parameter('align_servo_max_step_deg', 3.0)
        self.declare_parameter('align_error_tol_n', 0.02)

        # ---- Focuser params ----
        self.declare_parameter('focuser_bus', 9)
        self.declare_parameter('focus_start', 750)
        self.declare_parameter('focus_scan_step', 50)
        self.declare_parameter('focus_trim_step', 10)
        self.declare_parameter('focus_rel_drop_to_rescan', 0.40)
        self.declare_parameter('af_update_period_s', 0.25)

        # ---- Laser params ----
        self.declare_parameter('laser_pin_board', 22)
        self.declare_parameter('fire_duration', 10.0)  # seconds

        gp = self.get_parameter
        self.engine_path       = gp('engine_path').value
        self.device            = int(gp('device').value)
        self.imgsz             = int(gp('imgsz').value)
        self.conf_thresh       = float(gp('conf_thresh').value)
        self.half              = bool(gp('half').value)
        self.show_window       = bool(gp('show_window').value)
        self.target_label      = gp('target_label').value

        self.forward_speed     = float(gp('forward_speed').value)
        self.max_turn_speed    = float(gp('max_turn_speed').value)
        self.min_turn_speed    = float(gp('min_turn_speed').value)
        self.yaw_gain          = float(gp('yaw_gain').value)
        self.coverage_thresh   = float(gp('coverage_thresh').value)
        self.lost_timeout      = float(gp('lost_timeout').value)
        self.align_tol_n       = float(gp('align_tol_n').value)
        self.turn_speed_coupled= float(gp('turn_speed_coupled').value)

        self.turn_boost_speed  = float(gp('turn_boost_speed').value)
        self.turn_boost_duration = float(gp('turn_boost_duration').value)

        self.scan_min_deg      = float(gp('servo_scan_min_deg').value)
        self.scan_max_deg      = float(gp('servo_scan_max_deg').value)
        self.scan_step_deg     = float(gp('servo_scan_step_deg').value)
        self.servo_center_deg  = float(gp('servo_center_deg').value)
        self.servo_center_step = float(gp('servo_center_step_deg').value)
        self.servo_center_tol  = float(gp('servo_center_tol_deg').value)

        self.pca_bus           = int(gp('pca_bus').value)
        self.pca_addr          = int(gp('pca_addr').value)
        self.pca_freq_hz       = float(gp('pca_freq_hz').value)
        self.servo_channel     = int(gp('servo_channel').value)

        self.align_servo_gain  = float(gp('align_servo_gain_deg').value)
        self.align_servo_max_step = float(gp('align_servo_max_step_deg').value)
        self.align_error_tol_n = float(gp('align_error_tol_n').value)

        self.focuser_bus       = int(gp('focuser_bus').value)
        self.focus_start       = int(gp('focus_start').value)
        self.focus_scan        = int(gp('focus_scan_step').value)
        self.focus_trim_step   = int(gp('focus_trim_step').value)
        self.focus_rel_drop    = float(gp('focus_rel_drop_to_rescan').value)
        self.af_update_period  = float(gp('af_update_period_s').value)

        self.laser_pin         = int(gp('laser_pin_board').value)
        self.fire_duration     = float(gp('fire_duration').value)

        # ---- Publishers / GPIO ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.laser_pin, GPIO.OUT, initial=GPIO.LOW)

        # ---- YOLO ----
        self.get_logger().info(f"Loading YOLO engine from: {self.engine_path}")
        self.model = YOLO(self.engine_path, task="detect")

        # ---- Camera (IMX519) ----
        GST = (
            "nvarguscamerasrc sensor-id=0 !"
            "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=50/1,format=NV12 !"
            "nvvidconv ! video/x-raw,format=BGRx !"
            "videoconvert ! video/x-raw,format=BGR !"
            "appsink drop=true max-buffers=4 sync=false"
        )
        self.cap = cv2.VideoCapture(GST, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open IMX519 camera")

        self.frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1920
        self.frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 1080

        if self.show_window:
            cv2.namedWindow("enemy_approach_af", cv2.WINDOW_AUTOSIZE)

        # ---- Focuser ----
        self.focuser = Focuser(bus=self.focuser_bus)
        self.focus_pos = int(self.focus_start)
        self.focuser.set(Focuser.OPT_FOCUS, self.focus_pos)
        self.af_state = 1
        self.conf_max = 0.0
        self.ideal_focus = self.focus_pos
        self.past_conf = 0.0
        self.last_af_time = time.time()

        # ---- PCA9685 servo ----
        self.pca = PCA9685LowLevel(busnum=self.pca_bus,
                                   address=self.pca_addr,
                                   freq_hz=self.pca_freq_hz)
        self.servo_angle = self.servo_center_deg
        self._set_servo(self.servo_center_deg)
        self.scan_dir = 1

        # ---- FSM ----
        self.state = self.STATE_SEARCH
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.last_target_seen_time = None
        self.fire_start_time = None

        # Turn boost state
        self.last_turn_dir = 0  # -1, 0, +1
        self.turn_boost_end_time = 0.0

        self.timer = self.create_timer(0.0, self.loop_once)
        self.get_logger().info("EnemyApproachAutofocusNode initialized.")

    # ---- Servo helper ----
    def _set_servo(self, angle_deg: float):
        self.servo_angle = float(clamp(angle_deg, 0.0, 180.0))
        duty = angle_to_duty(self.servo_angle, self.pca_freq_hz)
        self.pca.set_pwm_duty(self.servo_channel, duty)

    # ---- YOLO helpers ----
    @staticmethod
    def _names(r):
        try:
            return r.names
        except Exception:
            return {}

    def select_target(self, r, frame_shape):
        if len(r.boxes) == 0:
            return None

        h, w = frame_shape[:2]
        try:
            names = self._names(r)
            cls = r.boxes.cls.detach().cpu().numpy().astype(int)
            confs = r.boxes.conf.detach().cpu().numpy()
            xywh = r.boxes.xywh.detach().cpu().numpy()
        except Exception as e:
            self.get_logger().warn(f"Error parsing YOLO boxes: {e}")
            return None

        # prefer specific label if present
        target_idxs = []
        if isinstance(names, dict) and self.target_label in names.values():
            for i, cid in enumerate(cls):
                if names.get(int(cid), "") == self.target_label:
                    target_idxs.append(i)
        if target_idxs:
            idxs = target_idxs
        else:
            idxs = [int(np.argmax(confs))]

        best_i = idxs[0]
        best_conf = confs[best_i]
        for i in idxs[1:]:
            if confs[i] > best_conf:
                best_conf = confs[i]
                best_i = i

        x, y, bw, bh = xywh[best_i]
        area_ratio = (bw * bh) / float(w * h)
        x_center_px = x
        x_center_n = x / float(w)

        return {
            "x_center_px": float(x_center_px),
            "x_center_n": float(x_center_n),
            "area_ratio": float(area_ratio),
            "conf": float(best_conf),
        }

    # ---- Motion helpers ----
    def send_cmd(self, v, w, label=None):
        now = time.time()

        if abs(w) > 1e-3:
            dir_sign = 1 if w > 0.0 else -1

            # Detect new turn start or direction change
            if self.last_turn_dir == 0 or dir_sign != self.last_turn_dir:
                self.turn_boost_end_time = now + self.turn_boost_duration

            self.last_turn_dir = dir_sign

            # Base magnitude (respect max_turn_speed)
            base_mag = min(abs(w), self.max_turn_speed)

            # Apply boost at start of turn, then enforce min_turn_speed
            if now <= self.turn_boost_end_time:
                mag = self.turn_boost_speed
            else:
                mag = max(base_mag, self.min_turn_speed)

            mag = min(mag, self.max_turn_speed)
            w = dir_sign * mag
        else:
            # No turning: reset boost state
            self.last_turn_dir = 0
            self.turn_boost_end_time = 0.0

        if abs(v - self.last_cmd_v) < 1e-3 and abs(w - self.last_cmd_w) < 1e-3:
            return
        self.last_cmd_v = v
        self.last_cmd_w = w

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)
        if label:
            self.get_logger().info(f"{label}: v={v:.2f}, w={w:.2f}")

    def stop(self, label="STOP"):
        self.send_cmd(0.0, 0.0, label)

    def laser_on(self):
        GPIO.output(self.laser_pin, GPIO.HIGH)
        self.get_logger().info("Laser ON")

    def laser_off(self):
        GPIO.output(self.laser_pin, GPIO.LOW)
        self.get_logger().info("Laser OFF")

    # ---- Autofocus ----
    def update_focus(self, has_target: bool, conf: float):
        now = time.time()
        if (now - self.last_af_time) < self.af_update_period:
            return
        self.last_af_time = now

        if not has_target:
            # continuous sweep when no target
            self.focus_pos += self.focus_scan
            if self.focus_pos > 1023:
                self.focus_pos = 0
            self.focus_pos = int(clamp(self.focus_pos, 0, 1023))
            try:
                self.focuser.set(Focuser.OPT_FOCUS, self.focus_pos)
            except Exception:
                pass
            self.af_state = 1
            self.conf_max = 0.0
            self.ideal_focus = self.focus_pos
            self.past_conf = 0.0
            return

        # Target present → small AF state machine
        if self.af_state == 1:
            if conf >= self.conf_max:
                self.conf_max = conf
                self.ideal_focus = self.focus_pos
            self.focus_pos -= self.focus_scan
            if self.focus_pos <= 0:
                self.focus_pos = int(self.ideal_focus)
                self.af_state = 2
        elif self.af_state == 2:
            self.focus_pos += self.focus_trim_step
            if conf <= self.past_conf - 0.01:
                self.focus_trim_step = -self.focus_trim_step
            if self.past_conf > 0.0 and conf < (self.past_conf * (1.0 - self.focus_rel_drop)):
                self.af_state = 1
                self.focus_pos = int(self.focus_start)
                self.conf_max = 0.0
                self.ideal_focus = self.focus_pos

        self.focus_pos = int(clamp(self.focus_pos, 0, 1023))
        self.past_conf = conf
        try:
            self.focuser.set(Focuser.OPT_FOCUS, self.focus_pos)
        except Exception:
            pass

    # ---- Main loop ----
    def loop_once(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Camera frame grab failed.")
            return

        now = time.time()

        results = self.model.predict(
            source=frame,
            device=self.device,
            imgsz=self.imgsz,
            conf=self.conf_thresh,
            half=self.half,
            verbose=False
        )
        r = results[0]

        det = self.select_target(r, frame.shape)
        has_target = det is not None and det["conf"] >= self.conf_thresh
        conf = det["conf"] if has_target else 0.0

        if has_target:
            self.last_target_seen_time = now

        if self.show_window:
            annotated = r.plot()
            h, w = frame.shape[:2]
            cv2.line(annotated, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
            if det is not None:
                txt = f"conf={det['conf']:.2f}, cov={det['area_ratio']:.2f}, focus={self.focus_pos}"
                cv2.putText(annotated, txt, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("enemy_approach_af", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Key 'q' → shutdown")
                self.cleanup()
                rclpy.shutdown()
                return

        # Always keep AF updated
        self.update_focus(has_target, conf)

        # ---- FSM ----
        if self.state == self.STATE_SEARCH:
            self.stop("SEARCH")

            # Servo sweep 30–150°
            next_angle = self.servo_angle + self.scan_step_deg * self.scan_dir
            if next_angle > self.scan_max_deg:
                next_angle = self.scan_max_deg
                self.scan_dir = -1
            elif next_angle < self.scan_min_deg:
                next_angle = self.scan_min_deg
                self.scan_dir = 1
            self._set_servo(next_angle)

            if has_target:
                self.get_logger().info("Target detected → ALIGN_COUPLED")
                self.state = self.STATE_ALIGN_COUPLED
                # Start ALIGN_COUPLED from current servo angle (no snap)
                self.stop("SEARCH→ALIGN_COUPLED")

        elif self.state == self.STATE_ALIGN_COUPLED:
            if not has_target:
                if (self.last_target_seen_time is None) or (now - self.last_target_seen_time > self.lost_timeout):
                    self.get_logger().info("Lost target → SEARCH")
                    self.state = self.STATE_SEARCH
                    self.stop("ALIGN_COUPLED→SEARCH")
                else:
                    self.stop("ALIGN_COUPLED (brief loss)")
                return

            # Only job here: bring servo to 90° while rover rotates opposite
            diff = self.servo_center_deg - self.servo_angle
            if abs(diff) <= self.servo_center_tol:
                # Close enough → snap to exactly 90 and move to DRIVE
                self._set_servo(self.servo_center_deg)
                self.get_logger().info("Servo reached 90° → DRIVE")
                self.state = self.STATE_DRIVE
                self.stop("ALIGN_COUPLED→DRIVE")
                return

            # Take a small step toward center
            step = clamp(diff, -self.servo_center_step, self.servo_center_step)
            new_angle = self.servo_angle + step
            self._set_servo(new_angle)

            # While servo is moving, rotate rover opposite direction
            if step > 0.0:
                # servo moving toward the right (angle increasing) → rover left
                w_cmd = self.turn_speed_coupled
            else:
                # servo moving toward the left → rover right
                w_cmd = -self.turn_speed_coupled

            self.send_cmd(0.0, w_cmd, "ALIGN_COUPLED (servo+rover)")

        elif self.state == self.STATE_DRIVE:
            if not has_target:
                if (self.last_target_seen_time is None) or (now - self.last_target_seen_time > self.lost_timeout):
                    self.get_logger().info("Lost target → SEARCH")
                    self.state = self.STATE_SEARCH
                    self.stop("DRIVE→SEARCH")
                else:
                    self.stop("DRIVE (brief loss)")
                return

            # Servo locked at 90° during DRIVE
            self._set_servo(self.servo_center_deg)

            area = det["area_ratio"]
            x_center_n = det["x_center_n"]
            error_n = x_center_n - 0.5

            if area >= self.coverage_thresh:
                self.get_logger().info("Coverage ≥ threshold → ALIGN_SERVO")
                self.state = self.STATE_ALIGN_SERVO
                self.stop("DRIVE→ALIGN_SERVO")
                return

            # Forward with P-controlled turn (boost logic in send_cmd)
            w_cmd = -self.yaw_gain * error_n
            w_cmd = clamp(w_cmd, -self.max_turn_speed, self.max_turn_speed)
            v_cmd = self.forward_speed
            self.send_cmd(v_cmd, w_cmd, "DRIVE")

        elif self.state == self.STATE_ALIGN_SERVO:
            # Rover stopped; servo alone pans to perfect center
            self.stop("ALIGN_SERVO")
            if not has_target:
                if (self.last_target_seen_time is None) or (now - self.last_target_seen_time > self.lost_timeout):
                    self.get_logger().info("Lost target in ALIGN_SERVO → SEARCH")
                    self.state = self.STATE_SEARCH
                    self.stop("ALIGN_SERVO→SEARCH")
                return

            x_center_n = det["x_center_n"]
            error_n = x_center_n - 0.5

            if abs(error_n) <= self.align_error_tol_n:
                self.get_logger().info("Target centered → FIRE")
                self.state = self.STATE_FIRE
                self.fire_start_time = now
                self.laser_on()
                return

            raw_step = -self.align_servo_gain * error_n
            step = clamp(raw_step, -self.align_servo_max_step, self.align_servo_max_step)
            self._set_servo(self.servo_angle + step)

        elif self.state == self.STATE_FIRE:
            self.stop("FIRE")
            if self.fire_start_time is None:
                self.fire_start_time = now
            elapsed = now - self.fire_start_time
            if elapsed >= self.fire_duration:
                self.get_logger().info("Fire complete → SEARCH")
                self.laser_off()
                self.state = self.STATE_SEARCH
                self.stop("FIRE→SEARCH")

        else:
            self.get_logger().warn(f"Unknown state {self.state}, resetting to SEARCH")
            self.state = self.STATE_SEARCH
            self.stop("Failsafe→SEARCH")

    # ---- Cleanup ----
    def cleanup(self):
        self.stop("Cleanup")
        try:
            self.laser_off()
        except Exception:
            pass
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        try:
            if self.show_window:
                cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            self._set_servo(self.servo_center_deg)
            time.sleep(0.1)
            self.pca.set_pwm_duty(self.servo_channel, 0.0)
        except Exception:
            pass
        try:
            GPIO.cleanup(self.laser_pin)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = EnemyApproachAutofocusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt → shutdown")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
