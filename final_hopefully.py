import os, time, collections, cv2, numpy as np

from ultralytics import YOLO
import Jetson.GPIO as GPIO


from Focuser import Focuser  # assuming you saved your previous class

GST = (
    "nvarguscamerasrc sensor-id=0 !"
    "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=50/1,format=NV12 !"
    "nvvidconv !"
    "video/x-raw,format=BGRx !"
    "videoconvert ! video/x-raw,format=BGR !"
    "appsink drop=true max-buffers=4 sync=false"
)


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

focuser = Focuser(bus=9)
state = 1
focus_pos = 750           
conf_max = 0.0
A = 10
past_conf = 0.0
batch_max_conf = 0.0
batch_conf = []  
error = .40  
ideal_focus = 0.0
focuser.set(Focuser.OPT_FOCUS, focus_pos)

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

        r = results[0]

        current_time = time.time()
            
        annotated = results[0].plot()  # or use frame_bgr to avoid plot() cost
        cv2.imshow(WIN, annotated)     # always refresh the window     
        
        if (0.1 <= (current_time - prev) < 0.25):

                    if len(r.boxes.conf) > 0:
                        batch_conf.append(float(r.boxes.conf.max()))
                    else:
                        batch_conf.append(0.0)

        if current_time - prev >= .25:
            batch_max_conf = np.mean(batch_conf)
            if np.isnan(batch_max_conf):
                 batch_max_conf = 0.0  
            else:
                batch_max_conf
              
            if(state == 1):       
                if (batch_max_conf >= conf_max):
                    conf_max = batch_max_conf
                    ideal_focus = focus_pos
                    print(f"New ideal focus: {ideal_focus} with conf: {conf_max}")
                prev = current_time
                batch_conf.clear()
                focus_pos -= 50             
                if (focus_pos <= 0):
                    focus_pos = ideal_focus
                    state = 2
                    print(f"The ideal focus: {ideal_focus} with conf: {conf_max}")
                 
            elif(state == 2):
                focus_pos -= A
                if batch_max_conf <= past_conf - 0.01:
                    A = A * -1
                    print(f"Reversing direction at focus: {focus_pos} with conf: {batch_max_conf}")
                if (batch_max_conf < (past_conf - (past_conf *error))):
                    state = 1
                    focus_pos = 750
                    conf_max = 0
                    ideal_focus = 0
                    print(f"Cannot find object returning to scan: Focus :{ideal_focus} with conf: {past_conf}")
                prev = current_time
                past_conf = batch_max_conf 
                batch_conf.clear()
              
            focuser.set(Focuser.OPT_FOCUS, focus_pos)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  
finally:
    cap.release()
    cv2.destroyAllWindows()