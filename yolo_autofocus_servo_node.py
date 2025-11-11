#!/usr/bin/env python3
# ROS 2 node: YOLO + IMX519 + Focuser + hardware PWM servo (Jetson Orin Nano)
# - Uses /sys/class/pwm for stable 50 Hz servo pulses (recommended on Jetson)
# - Slow servo update cadence to diagnose jitter
# - Live OpenCV preview with overlays
# Run:
#   source /opt/ros/humble/setup.bash
#   export FASTDDS_TRANSPORT_SHARED_MEM=off
#   cd ~/Desktop/ros2_mapping_project && export PYTHONPATH=$PWD:$PYTHONPATH
#   python3 yolo_autofocus_servo_node.py --ros-args -p show_window:=true \
#     -p engine_path:="'/home/team4/Documents/camera/green_specific.engine'" \
#     -p pwm_chip:=0 -p pwm_channel:=0 -p servo_pin_name:="'PIN33'"

import os, sys, time, glob
sys.path.append(os.path.dirname(__file__))

import cv2, numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32

from Focuser import Focuser

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

# ---------- Hardware PWM helper (sysfs) ----------
class HwPWM:
    """
    Minimal /sys/class/pwm wrapper.
    period_ns: e.g. 20_000_000 for 50 Hz
    duty_ns:   between ~1_000_000–2_000_000 for servos
    """
    def __init__(self, chip:int, channel:int, period_ns:int=20_000_000):
        self.chip = chip
        self.channel = channel
        self.base = f"/sys/class/pwm/pwmchip{chip}"
        self.pwm_path = f"{self.base}/pwm{channel}"
        if not os.path.isdir(self.base):
            # try to auto-pick a chip that exists
            chips = sorted(glob.glob("/sys/class/pwm/pwmchip*"))
            if not chips:
                raise RuntimeError("No pwmchip found under /sys/class/pwm (enable PWM in Jetson-IO and reboot).")
            self.base = chips[0]
            self.chip = int(os.path.basename(self.base).replace("pwmchip",""))
            self.pwm_path = f"{self.base}/pwm{channel}"

        # export channel if needed
        if not os.path.isdir(self.pwm_path):
            with open(f"{self.base}/export","w") as f: f.write(str(channel))
            # kernel creates nodes a moment later
            for _ in range(20):
                if os.path.isdir(self.pwm_path): break
                time.sleep(0.01)

        # if already enabled, disable before reconfiguring
        self.disable()
        # set period
        with open(f"{self.pwm_path}/period","w") as f: f.write(str(period_ns))
        # default duty to neutral 1.5 ms
        with open(f"{self.pwm_path}/duty_cycle","w") as f: f.write(str(1_500_000))
        self.enable(True)

        self.period_ns = period_ns
        self._last_set_ns = 1_500_000

    def enable(self, on:bool):
        with open(f"{self.pwm_path}/enable","w") as f: f.write("1" if on else "0")

    def disable(self):
        try:
            with open(f"{self.pwm_path}/enable","w") as f: f.write("0")
        except Exception:
            pass

    def angle_to_duty_ns(self, angle_deg:float)->int:
        a = clamp(float(angle_deg), 0.0, 180.0)
        # 1.0ms at 0°, 2.0ms at 180°
        return int(1_000_000 + (a/180.0)*1_000_000)

    def set_angle(self, angle_deg:float):
        duty_ns = self.angle_to_duty_ns(angle_deg)
        if duty_ns != self._last_set_ns:
            with open(f"{self.pwm_path}/duty_cycle","w") as f: f.write(str(duty_ns))
            self._last_set_ns = duty_ns

    def close(self):
        # park at neutral briefly
        try:
            with open(f"{self.pwm_path}/duty_cycle","w") as f: f.write(str(1_500_000))
            time.sleep(0.05)
            self.disable()
        except Exception:
            pass


class YOLOAutofocusServoNode(Node):
    def __init__(self):
        super().__init__('yolo_autofocus_servo')

        # ---- Params ----
        self.declare_parameter('engine_path', '/home/team4/Documents/camera/green_specific.engine')
        self.declare_parameter('device', 0)
        self.declare_parameter('imgsz', 704)
        self.declare_parameter('conf_thresh', 0.25)
        self.declare_parameter('half', True)
        self.declare_parameter('target_label', 'enemy')

        self.declare_parameter('show_window', True)
        self.declare_parameter('draw_crosshair', True)
        self.declare_parameter('gst_pipeline',
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=50/1,format=NV12 ! "
            "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink drop=true max-buffers=4 sync=false")

        # Lock/track
        self.declare_parameter('lock_conf', 0.70)
        self.declare_parameter('lock_time_s', 0.50)
        self.declare_parameter('lost_timeout_s', 3.0)

        # Hardware PWM (recommend PIN33 → pwmchip for 32c0000.pwm)
        self.declare_parameter('pwm_chip', 0)        # set to the pwmchip index you have
        self.declare_parameter('pwm_channel', 0)     # typically 0
        self.declare_parameter('servo_pin_name', 'PIN33')  # for your own reference/logging
        self.declare_parameter('servo_update_period_s', 0.5)  # move only every 0.5 s
        self.declare_parameter('min_move_deg', 2.0)
        self.declare_parameter('max_step_deg', 3.0)

        # Pan controller
        self.declare_parameter('kp_deg_per_px', 0.015)
        self.declare_parameter('deadzone_px', 32.0)
        self.declare_parameter('scan_step_deg', 0.5)
        self.declare_parameter('ema_alpha', 0.2)

        # Focuser
        self.declare_parameter('focuser_bus', 9)
        self.declare_parameter('focus_start', 750)
        self.declare_parameter('focus_scan_step', 50)
        self.declare_parameter('focus_trim_step', 10)
        self.declare_parameter('focus_rel_drop_to_rescan', 0.40)

        # ---- Read params ----
        gp = self.get_parameter
        self.engine_path = gp('engine_path').get_parameter_value().string_value
        self.device = int(gp('device').value)
        self.imgsz = int(gp('imgsz').value)
        self.conf_thresh = float(gp('conf_thresh').value)
        self.half = bool(gp('half').value)
        self.target_label = gp('target_label').get_parameter_value().string_value

        self.show_window = bool(gp('show_window').value)
        self.draw_crosshair = bool(gp('draw_crosshair').value)
        self.gst = gp('gst_pipeline').get_parameter_value().string_value

        self.lock_conf = float(gp('lock_conf').value)
        self.lock_time_s = float(gp('lock_time_s').value)
        self.lost_timeout_s = float(gp('lost_timeout_s').value)

        self.pwm_chip = int(gp('pwm_chip').value)
        self.pwm_channel = int(gp('pwm_channel').value)
        self.servo_pin_name = gp('servo_pin_name').get_parameter_value().string_value
        self.servo_update_period = float(gp('servo_update_period_s').value)
        self.min_move_deg = float(gp('min_move_deg').value)
        self.max_step_deg = float(gp('max_step_deg').value)

        self.kp = float(gp('kp_deg_per_px').value)
        self.deadzone_px = float(gp('deadzone_px').value)
        self.scan_step_deg = float(gp('scan_step_deg').value)
        self.ema_alpha = float(gp('ema_alpha').value)

        self.focuser_bus = int(gp('focuser_bus').value)
        self.focus_pos = int(gp('focus_start').value)
        self.focus_scan = int(gp('focus_scan_step').value)
        self.focus_trim = int(gp('focus_trim_step').value)
        self.focus_drop = float(gp('focus_rel_drop_to_rescan').value)

        # ---- Camera ----
        self.cap = cv2.VideoCapture(self.gst, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open IMX519 via GStreamer")
        self.frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1920
        self.frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 1080
        self.img_center_x = self.frame_w/2.0

        # ---- YOLO ----
        self.model = YOLO(self.engine_path, task="detect")

        # ---- Focuser ----
        self.focuser = Focuser(bus=self.focuser_bus)
        self.focuser.set(Focuser.OPT_FOCUS, self.focus_pos)

        # ---- Hardware PWM servo ----
        # 50 Hz period = 20,000,000 ns
        self.pwm = HwPWM(self.pwm_chip, self.pwm_channel, period_ns=20_000_000)
        self.pan_angle = 90.0
        self.pwm.set_angle(self.pan_angle)
        self.last_servo_update = time.time()

        # ---- State ----
        self.state = 1
        self.scan_dir = 1
        self.last_batch_t = time.time()
        self.batch_conf = []
        self.last_seen_t = None
        self.object_seen_time = 0.0
        self.object_lost_time = 0.0
        self.conf_max = 0.0
        self.ideal_focus = self.focus_pos
        self.past_conf = 0.0
        self.trim_step = self.focus_trim
        self.xc_ema = None

        if self.show_window:
            cv2.namedWindow("YOLO AF", cv2.WINDOW_AUTOSIZE)

        # Publishers
        self.pub_locked = self.create_publisher(Bool, '/enemy_lock', 10)
        self.pub_angle  = self.create_publisher(Float32, '/servo/angle', 10)
        self.pub_focus  = self.create_publisher(Int32, '/focuser/position', 10)
        self.pub_conf   = self.create_publisher(Float32, '/enemy/conf', 10)

        # Main loop
        self.timer = self.create_timer(0.0, self.loop_once)
        self.get_logger().info(f"Started. Using {self.servo_pin_name} via /sys/class/pwm/pwmchip{self.pwm_chip}/pwm{self.pwm_channel}")

    # ---------- Helpers ----------
    @staticmethod
    def _names(r):
        try: return r.names
        except: return {}

    def select_target(self, r):
        if len(r.boxes)==0: return None, 0.0
        try:
            names = self._names(r)
            if isinstance(names, dict) and self.target_label in names.values():
                cls_id = [k for k,v in names.items() if v==self.target_label][0]
                cls = r.boxes.cls.detach().cpu().numpy().astype(int)
                mask = (cls==cls_id)
                if mask.any():
                    idxs = np.where(mask)[0]
                    confs = r.boxes.conf.detach().cpu().numpy()[idxs]
                    bi = idxs[int(np.argmax(confs))]
                    return float(r.boxes.xywh[bi,0]), float(r.boxes.conf[bi])
        except Exception:
            pass
        i = int(r.boxes.conf.argmax())
        return float(r.boxes.xywh[i,0]), float(r.boxes.conf[i])

    def maybe_update_servo(self, target_angle: float):
        now = time.time()
        if (now - self.last_servo_update) < self.servo_update_period:
            return
        delta = target_angle - self.pan_angle
        if abs(delta) < self.min_move_deg:
            return
        step = clamp(delta, -self.max_step_deg, self.max_step_deg)
        self.pan_angle = clamp(self.pan_angle + step, 0.0, 180.0)
        try:
            self.pwm.set_angle(self.pan_angle)
        except Exception as e:
            self.get_logger().warn(f"PWM set failed: {e}")
        self.last_servo_update = now
        self.get_logger().info(f"servo→ {self.pan_angle:.1f}° (req:{target_angle:.1f}°, step:{step:.1f}°)")

    # ---------- Main loop ----------
    def loop_once(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        results = self.model.predict(
            source=frame, device=self.device, imgsz=self.imgsz,
            conf=self.conf_thresh, half=self.half, verbose=False
        )
        r = results[0]

        # Select target + smooth
        x_center, inst_conf = self.select_target(r) if len(r.boxes)>0 else (None,0.0)
        if x_center is not None:
            self.xc_ema = x_center if self.xc_ema is None else \
                          (self.ema_alpha*x_center + (1.0-self.ema_alpha)*self.xc_ema)

        # Confidence batching (0.25 s)
        now = time.time()
        batch_max_conf = None
        self.batch_conf.append(inst_conf)
        if now - self.last_batch_t >= 0.25:
            m = float(np.mean(self.batch_conf)) if self.batch_conf else 0.0
            if not np.isfinite(m): m = 0.0
            self.batch_conf.clear()
            self.last_batch_t = now
            batch_max_conf = m

        # Timers + AF FSM
        if batch_max_conf is not None:
            if batch_max_conf >= self.lock_conf:
                if self.last_seen_t is None:
                    self.last_seen_t = now; self.object_seen_time = 0.0; self.object_lost_time = 0.0
                else:
                    self.object_seen_time = now - self.last_seen_t; self.object_lost_time = 0.0
            else:
                if self.last_seen_t is not None:
                    self.object_lost_time += 0.25
                    if self.object_lost_time >= self.lost_timeout_s:
                        self.last_seen_t = None; self.object_seen_time = 0.0
                else:
                    self.object_lost_time += 0.25

            # AF FSM
            if self.state == 1:
                if batch_max_conf >= self.conf_max:
                    self.conf_max = batch_max_conf; self.ideal_focus = self.focus_pos
                self.focus_pos -= self.focus_scan
                if self.focus_pos <= 0:
                    self.focus_pos = int(self.ideal_focus); self.state = 2

            elif self.state == 2:
                self.focus_pos += self.focus_trim
                if batch_max_conf <= self.past_conf - 0.01:
                    self.focus_trim = -self.focus_trim
                if (self.past_conf>0 and batch_max_conf < (self.past_conf*(1.0-self.focus_drop))) or \
                   (self.object_lost_time >= self.lost_timeout_s):
                    self.state = 1
                    self.focus_pos = int(self.get_parameter('focus_start').value)
                    self.conf_max = 0.0; self.ideal_focus = 0
                if (self.object_seen_time >= self.lock_time_s) and (batch_max_conf >= self.lock_conf):
                    self.state = 3
                self.past_conf = batch_max_conf

            elif self.state == 3:
                self.focus_pos = int(clamp(self.focus_pos, 0, 1023))
                if self.object_lost_time >= self.lost_timeout_s:
                    self.state = 1
                self.past_conf = batch_max_conf

            self.focus_pos = int(clamp(self.focus_pos, 0, 1023))
            try:
                self.focuser.set(Focuser.OPT_FOCUS, self.focus_pos)
            except Exception as e:
                self.get_logger().warn(f"Focuser set failed: {e}")

            self.pub_focus.publish(Int32(data=self.focus_pos))
            self.pub_conf.publish(Float32(data=batch_max_conf))

        # Pan target (apply slowly via maybe_update_servo)
        if self.state != 3 or self.xc_ema is None:
            nxt = self.pan_angle + self.scan_step_deg*self.scan_dir
            if (nxt > 180.0) or (nxt < 0.0):
                self.scan_dir *= -1
                nxt = self.pan_angle + self.scan_step_deg*self.scan_dir
            target_angle = clamp(nxt, 0.0, 180.0)
        else:
            err_px = self.xc_ema - self.img_center_x
            if abs(err_px) <= self.deadzone_px:
                target_angle = self.pan_angle
            else:
                target_angle = clamp(self.pan_angle + self.kp*err_px, 0.0, 180.0)

        self.maybe_update_servo(target_angle)

        # Live preview (always on if show_window==True)
        if self.show_window:
            try:
                viz = results[0].plot()
                if self.draw_crosshair:
                    cx, cy = int(self.img_center_x), int(self.frame_h/2)
                    cv2.drawMarker(viz, (cx, cy), (0,255,255), markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2)
                cv2.putText(viz, f"lock:{self.state==3} state:{self.state} angle:{self.pan_angle:.1f}",
                            (12,24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("YOLO AF", viz)
                cv2.waitKey(1)
            except Exception:
                pass

        # pubs
        self.pub_locked.publish(Bool(data=(self.state==3)))
        self.pub_angle.publish(Float32(data=float(self.pan_angle)))

    def destroy_node(self):
        try: self.pwm.close()
        except: pass
        try: self.cap.release()
        except: pass
        try: cv2.destroyAllWindows()
        except: pass
        super().destroy_node()


def main():
    rclpy.init()
    node = YOLOAutofocusServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
