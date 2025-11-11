# save as full_on_test.py and run: python3 full_on_test.py
import time, Jetson.GPIO as GPIO
ENA, ENB = 15, 32
IN1, IN2, IN3, IN4 = 23, 19, 21, 18  # A side uses IN1/IN2, B side uses IN3/IN4

GPIO.setmode(GPIO.BOARD)
for p in (ENB, ENA, IN1, IN2, IN3, IN4):
    GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

# forward both sides
GPIO.output(IN4, GPIO.HIGH); 
GPIO.output(IN3, GPIO.LOW);   # A forward
GPIO.output(IN1, GPIO.LOW); 
GPIO.output(IN2, GPIO.HIGH);   # B forward
GPIO.output(ENB, GPIO.HIGH); 
GPIO.output(ENA, GPIO.HIGH)  # FULL ON

print("Full power for 5s…")
time.sleep(5)

# stop (coast)
GPIO.output(ENA, GPIO.LOW); GPIO.output(ENB, GPIO.LOW)
GPIO.cleanup()
