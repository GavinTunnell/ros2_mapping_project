#!/usr/bin/env python3
# manual_motor_test.py  —  Jetson Orin Nano keyboard teleop for H-bridge
# Uses BOARD numbering. Run with sudo.
import sys, termios, tty, time
import Jetson.GPIO as GPIO

# ==== USER PINS (BOARD numbering) ====
ENA = 15   # PWM enable for Left side (A)
ENB = 32   # PWM enable for Right side (B)
A_IN1 = 19 # Left side input #1  (your IN2)
A_IN2 = 18 # Left side input #2  (your IN4)
B_IN1 = 23 # Right side input #1 (your IN1)
B_IN2 = 21 # Right side input #2 (your IN3)

# If "forward" is backwards on your wiring, flip these:
LEFT_INVERT  = False
RIGHT_INVERT = False

PWM_FREQ_HZ = 500    # Software PWM; keep modest for smoother duty
START_DUTY  = 40     # % duty to begin with (adjust up if motors stall)
STEP_DUTY   = 10     # % step for [ / ]

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    for p in (A_IN1, A_IN2, B_IN1, B_IN2, ENA, ENB):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
    pwm_left  = GPIO.PWM(ENA, PWM_FREQ_HZ)
    pwm_right = GPIO.PWM(ENB, PWM_FREQ_HZ)
    pwm_left.start(0)
    pwm_right.start(0)
    return pwm_left, pwm_right

def brake():
    # Both inputs HIGH = brake (most L298N-style drivers)
    GPIO.output(A_IN1, GPIO.HIGH); GPIO.output(A_IN2, GPIO.HIGH)
    GPIO.output(B_IN1, GPIO.HIGH); GPIO.output(B_IN2, GPIO.HIGH)

def coast():
    # Both inputs LOW = coast
    GPIO.output(A_IN1, GPIO.LOW); GPIO.output(A_IN2, GPIO.LOW)
    GPIO.output(B_IN1, GPIO.LOW); GPIO.output(B_IN2, GPIO.LOW)

def set_left(dir_fwd: int, duty: int, pwm_left):
    # dir_fwd: +1 forward, -1 backward, 0 stop
    if dir_fwd == 0:
        brake()
        pwm_left.ChangeDutyCycle(0)
        return
    fwd = (dir_fwd > 0)
    if LEFT_INVERT: fwd = not fwd
    if fwd:
        GPIO.output(A_IN1, GPIO.HIGH); GPIO.output(A_IN2, GPIO.LOW)
    else:
        GPIO.output(A_IN1, GPIO.LOW);  GPIO.output(A_IN2, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(max(0, min(100, duty)))

def set_right(dir_fwd: int, duty: int, pwm_right):
    if dir_fwd == 0:
        brake()
        pwm_right.ChangeDutyCycle(0)
        return
    fwd = (dir_fwd > 0)
    if RIGHT_INVERT: fwd = not fwd
    if fwd:
        GPIO.output(B_IN1, GPIO.HIGH); GPIO.output(B_IN2, GPIO.LOW)
    else:
        GPIO.output(B_IN1, GPIO.LOW);  GPIO.output(B_IN2, GPIO.HIGH)
    pwm_right.ChangeDutyCycle(max(0, min(100, duty)))

def stop_all(pwm_left, pwm_right):
    brake()
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

def print_help(duty):
    print(
        f"""
Keyboard teleop (duty={duty}%):
  w/s  : forward / backward (both)
  a/d  : spin left / spin right (pivot)
  q/e  : arc left / arc right  (L/R 70/40 duty)
  1/2  : LEFT wheel forward/back (solo)
  9/0  : RIGHT wheel forward/back (solo)
  [ / ]: duty - / +
  space: STOP (brake)
  h    : help
  ESC  : quit
"""
    )

def main():
    pwm_left, pwm_right = setup()
    duty = START_DUTY
    print_help(duty)
    print(">>> Press keys… (run with: sudo -E python3 manual_motor_test.py)")

    try:
        while True:
            ch = getch()
            if ch == '\x1b':  # ESC
                break
            elif ch in ('h', 'H'):
                print_help(duty)
            elif ch == ' ':
                stop_all(pwm_left, pwm_right)
                print("STOP")
            elif ch == '[':
                duty = max(0, duty - STEP_DUTY); print(f"duty -> {duty}%")
            elif ch == ']':
                duty = min(100, duty + STEP_DUTY); print(f"duty -> {duty}%")

            # Whole-robot motions
            elif ch in ('w', 'W'):
                set_left(+1, duty, pwm_left); set_right(+1, duty, pwm_right); print("FWD")
            elif ch in ('s', 'S'):
                set_left(-1, duty, pwm_left); set_right(-1, duty, pwm_right); print("BACK")
            elif ch in ('a', 'A'):
                set_left(-1, duty, pwm_left); set_right(+1, duty, pwm_right); print("SPIN LEFT")
            elif ch in ('d', 'D'):
                set_left(+1, duty, pwm_left); set_right(-1, duty, pwm_right); print("SPIN RIGHT")
            elif ch in ('q', 'Q'):
                set_left(+1, max(0, duty), pwm_left)
                set_right(+1, max(0, int(duty*0.6)), pwm_right)
                print("ARC LEFT")
            elif ch in ('e', 'E'):
                set_left(+1, max(0, int(duty*0.6)), pwm_left)
                set_right(+1, max(0, duty), pwm_right)
                print("ARC RIGHT")

            # Individual wheel jog
            elif ch == '1':
                set_left(+1, duty, pwm_left); set_right(0, 0, pwm_right); print("LEFT FWD")
            elif ch == '2':
                set_left(-1, duty, pwm_left); set_right(0, 0, pwm_right); print("LEFT BACK")
            elif ch == '9':
                set_left(0, 0, pwm_left); set_right(+1, duty, pwm_right); print("RIGHT FWD")
            elif ch == '0':
                set_left(0, 0, pwm_left); set_right(-1, duty, pwm_right); print("RIGHT BACK")

            # ignore other keys
    except KeyboardInterrupt:
        pass
    finally:
        stop_all(pwm_left, pwm_right)
        pwm_left.stop(); pwm_right.stop()
        GPIO.cleanup()
        print("Clean exit.")

if __name__ == "__main__":
    main()
