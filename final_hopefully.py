import os, time, collections, cv2, numpy as np,smbus
from ultralytics import YOLO
import Jetson.GPIO as GPIO
from Focuser import Focuser  # assuming you saved your previous class
import math
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

def sit_down_bitch(active):
    if not active:
        motors = 0
    else:
        motors = 1
    return motors

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
            # fully on
            self.set_pwm_raw(channel, 0, 4095)
        else:
            off = int(round(duty * 4095))
            off = max(1, min(4094, off))
            self.set_pwm_raw(channel, 0, off)

    def set_pin_digital(self, channel: int, high: bool):
        self.set_pwm_duty(channel, 1.0 if high else 0.0)

def angle_to_duty(angle, freq_hz=50.0):
    MIN_US = 500
    MAX_US = 2500
    angle = max(0, min(180, angle))
    us = MIN_US + (MAX_US - MIN_US) * angle / 180.0
    period_us = 1_000_000.0 / freq_hz
    duty = us / period_us    # fraction of total period
    return duty              # e.g. 0.075 for 7.5%

# ---------- PCA9685 instance ---------- 
#Contains Bus number, I2C address, and frequency in Hz
pca = PCA9685LowLevel(busnum=1, address=0x42, freq_hz=50.0)

# ---------- Jetson Nano Camera GStreamer Pipeline ----------
GST = (
    "nvarguscamerasrc sensor-id=0 !"
    "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=50/1,format=NV12 !"
    "nvvidconv !"
    "video/x-raw,format=BGRx !"
    "videoconvert ! video/x-raw,format=BGR !"
    "appsink drop=true max-buffers=4 sync=false"
)

SERVO_PIN = 22

GPIO.setmode(GPIO.BOARD)            # Sets up Laser GPIO and sets output to LOW
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.output(SERVO_PIN, GPIO.LOW) 

# -------- Model config --------
ENGINE_PATH = "green_specific.engine"   # your TensorRT engine
DEVICE      = 0
IMGSZ       = 704
CONF        = 0.25
HALF        = True

# -------- Load engine (explicit task to silence warning) --------
model = YOLO(ENGINE_PATH, task="detect")

# -------- Camera --------
cap = cv2.VideoCapture(GST, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise SystemExit("Failed to open camera.")

WIN = "IMX519 YOLOv8 TensorRT"
cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

times = collections.deque(maxlen=60)
prev  = time.time()

# ---------- State variables ----------
focuser = Focuser(bus=10)

# State 1 variables
state = 1
focus_pos = 300           
conf_max = 0.0
servo_offset = 0        #Adjust servo offset if needed
acquiring = True
object_detected = False  #Object detection fields
angle = 40
batch_conf = []  

# State 2 variables
focus_positions = list(range(650, 0, -50))  # [750, 700, ..., 50]
current_index = 0
ideal_focus = 300
object_detected_once = False

# State 3 Variables
acquire_tol = 150          # px: stay in sweep until |error| <= this
track_tol   = 10           # px: no movement if within this band (deadband)
direction = 1

# State 4 Variables
past_conf = 0.0
error = .30  
A = 5  # focus step size (change if you want faster/slower focus sweeps)

# State 5 Variables
CENTER_MIN = 0
CENTER_MAX = 180
px_cushion   = 30          # cushion in pixels around image center
slow_step    = 0.5         # degrees per nudge to keep motion slow

# State 6 Variables
laser_toggle_count = 0

#Intialize motors for nav2
sit_down_bitch(1)

#Sets initial servo position
duty = angle_to_duty(angle)
pca.set_pwm_duty(0, duty)

time.sleep(1)

try:
    while True: 

        ok, frame_bgr = cap.read()
        if not ok:
            break

        results = model.predict(
            source=frame_bgr,
            device=DEVICE,
            imgsz=IMGSZ,
            conf=CONF,
            half=HALF,
            verbose=False
        )

        frame_width = frame_bgr.shape[1]  
        center_x = frame_width / 2
        tolerance = 20  

        r = results[0]

        current_time = time.time()

        if (0.1 <= (current_time - prev) < 0.25):
            if len(r.boxes.conf) > 0:
                batch_conf.append(float(r.boxes.conf.max()))
            else:
                batch_conf.append(0.0)

        # top-conf x_center in pixels
        x_center = float(r.boxes.xywh[r.boxes.conf.argmax(), 0]) if len(r.boxes) > 0 else None

        # normalized center for TOY SOLDIER (prefer), else top-conf box
        x_center_n = None
        
        if len(r.boxes) > 0:
            try:
                names = r.names if hasattr(r, "names") else model.names
                cls_ids = r.boxes.cls.cpu().numpy().astype(int)
                xywhn   = r.boxes.xywhn.cpu().numpy()
                confs   = r.boxes.conf.cpu().numpy()

                # try to pick "toy soldier" if present
                target_idx = None
                if names is not None:
                    for i, cid in enumerate(cls_ids):
                        if str(names.get(int(cid), "")).lower() == "toy soldier":
                            if target_idx is None or confs[i] > confs[target_idx]:
                                target_idx = i
                # fallback to highest confidence
                if target_idx is None:
                    target_idx = int(np.argmax(confs))

                x_center_n = float(xywhn[target_idx, 0])  # normalized 0..1
            except Exception:
                if x_center is not None and frame_width > 0:
                    x_center_n = float(x_center / frame_width)

        annotated = results[0].plot()
        cv2.imshow(WIN, annotated)

        # IMPORTANT: let OpenCV process GUI events
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        
        # ---------- Per-frame state logic ----------
        if state == 1:
            acquiring = True
            if angle > 180:
                angle = 180
                direction = -1              
            elif angle < 40:
                angle = 40
                direction = 1

            # sweep
            angle += 1 * direction

            duty = angle_to_duty(angle + servo_offset)
            pca.set_pwm_duty(0, duty)

            if object_detected:
                state = 2

        if state == 3:
            if x_center is not None:
                error_pixels = center_x - x_center  # + => target left of center

                if acquiring and abs(error_pixels) > acquire_tol:
                    # keep sweeping in the current direction
                    angle += 1 * direction
                else:
                    acquiring = False
                    if abs(error_pixels) > track_tol:
                        state = 4  # switch to fine-tracking state
            else:
                acquiring = True
                angle += 1 * direction
                
            duty = angle_to_duty(angle + servo_offset)
            pca.set_pwm_duty(0, duty)
        
        if state == 5:
            if x_center_n is not None and frame_width > 0:
                margin_n = float(px_cushion) / float(frame_width)
                # nudge slowly toward the center
                if x_center_n < (0.5 - margin_n):
                    angle += slow_step     # target is left in image → turn left
                elif x_center_n > (0.5 + margin_n):
                    angle -= slow_step     # target is right in image → turn right
                # clamp to mechanical limits
                if angle > CENTER_MAX:
                    angle = CENTER_MAX
                elif angle < CENTER_MIN:
                    angle = CENTER_MIN
            # if no detection, just hold current angle
            duty = angle_to_duty(angle + servo_offset)
            pca.set_pwm_duty(0, duty)
        
        # ---------- 0.25 s logic ----------
        if current_time - prev >= .25:
            batch_max_conf = np.mean(batch_conf) if batch_conf else 0.0
            if np.isnan(batch_max_conf) or batch_max_conf < 0.5:
                batch_max_conf = 0.0

            if state == 1:
                if batch_max_conf >= 0.5:
                    focus_pos = 750
                    object_detected = True
                    acquiring = True
                    sit_down_bitch(0)
                    print(f"Object detected with confidence {batch_max_conf:.2f}, switching to state 1→2 for focusing.")
                    state = 2
                    current_index = 0
                    conf_max = 0.0
                    ideal_focus = focus_pos

            if state == 2:  # Autofocus sweep state
                if batch_max_conf >= conf_max:
                    conf_max = batch_max_conf
                    ideal_focus = focus_pos
                    print(f"New ideal focus: {ideal_focus} with conf: {conf_max}")

                # move to next focus position safely
                current_index += 1
                if current_index >= len(focus_positions):
                    # finished full sweep
                    if conf_max < 0.5:
                        object_detected_once = False
                        focus_pos = 300
                        current_index = 0
                        state = 1
                        print("Sweep done, low confidence. Returning to scan.")
                    else:
                        object_detected_once = False
                        focus_pos = ideal_focus
                        print(
                            f"Completed first full sweep after detection. "
                            f"Best focus: {ideal_focus} with conf: {conf_max}"
                        )
                        current_index = 0
                        state = 3
                else:
                    focus_pos = focus_positions[current_index]

            focuser.set(Focuser.OPT_FOCUS, focus_pos)

            if state == 4:
                print(state)
                focus_pos -= A
                if batch_max_conf <= past_conf - 0.01:
                    A = -A
                    print(f"Reversing direction at focus: {focus_pos} with conf: {batch_max_conf}")
                if batch_max_conf < (past_conf - (past_conf * error)):
                    state = 1
                    focus_pos = 300
                    conf_max = 0
                    ideal_focus = 0
                    print(f"Cannot refine focus, returning to scan. Last good conf: {past_conf}")
                if batch_max_conf >= past_conf and batch_max_conf >= 0.5:
                    state = 5
                    print(f"Object in focus with sufficient confidence. Holding focus at: {focus_pos} with conf: {batch_max_conf}")
            
            if state == 5:
                # dwell here some time before firing laser
                laser_toggle_count += 1
                if laser_toggle_count > 16:  # ~4 s at 0.25s per tick
                    laser_toggle_count = 0
                    state = 6

            if state == 6:
                # LASER ON window
                if laser_toggle_count == 0:
                    GPIO.output(SERVO_PIN, GPIO.HIGH)  # turn laser ON once
                laser_toggle_count += 1
                print(6)
                if laser_toggle_count >= 40:  # ~10 s ON time
                    GPIO.output(SERVO_PIN, GPIO.LOW)   # turn laser OFF
                    laser_toggle_count = 0
                    state = 7

            if state == 7:
                # cool-down / re-enable motors, keep laser OFF
                laser_toggle_count += 1
                sit_down_bitch(1)  # re-enable motors
                if laser_toggle_count >= 40:  # ~10 s cool-down
                    focus_pos = 300
                    laser_toggle_count = 0
                    GPIO.output(SERVO_PIN, GPIO.LOW)   # ensure laser stays OFF
                    state = 1
                    conf_max = 0.0
                    acquiring = True
                    object_detected = False
                    angle = 30

            # update past_conf for next tick (used in state 4)
            past_conf = batch_max_conf

            prev = current_time
            batch_conf.clear()

finally:
    cap.release()
    cv2.destroyAllWindows()
