# /tmp/pinpeek.py
import Jetson.GPIO as GPIO, time
GPIO.setmode(GPIO.BOARD); GPIO.setwarnings(False)
for p in (12,16,7,11): GPIO.setup(p, GPIO.IN)   # 12/16 = left, 7/11 = right
print("Press Ctrl+C to stop")
try:
    while True:
        la, lb = GPIO.input(12), GPIO.input(16)
        ra, rb = GPIO.input(7),  GPIO.input(11)
        print(f"L(A,B)=({la},{lb})   R(A,B)=({ra},{rb})")
        time.sleep(0.05)
finally:
    GPIO.cleanup()
