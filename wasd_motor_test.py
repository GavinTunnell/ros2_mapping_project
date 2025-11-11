#!/usr/bin/env python3
import sys, termios, tty, time
try:
    import Jetson.GPIO as GPIO
    HAS_GPIO = True
except Exception:
    HAS_GPIO = False

# ---- Pin map (BOARD numbers) ----
ENA = 15   # side A enable (PWM)
ENB = 32   # side B enable (PWM)
IN1 = 23   # A dir1
IN2 = 19   # A dir2
IN3 = 21   # B dir1
IN4 = 18   # B dir2

PWM_HZ = 1000
MIN_DUTY = 20.0  # dead-zone lift
duty = 40.0      # start duty

def kbhit():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def set_side(speed, inA, inB, pwm):  # speed in [-1..1]
    if not HAS_GPIO:
        return
    if abs(speed) < 1e-3:
        # coast (both low)
        GPIO.output(inA, GPIO.LOW); GPIO.output(inB, GPIO.LOW)
        pwm.ChangeDutyCycle(0.0)
        return
    if speed > 0:
        # forward
        GPIO.output(inA, GPIO.HIGH); GPIO.output(inB, GPIO.LOW)
    else:
        # reverse
        GPIO.output(inA, GPIO.LOW);  GPIO.output(inB, GPIO.HIGH)
    pwm.ChangeDutyCycle(max(MIN_DUTY, min(100.0, abs(speed) * 100.0)))

def brake():
    if not HAS_GPIO: return
    for p in (IN1, IN2, IN3, IN4):
        GPIO.output(p, GPIO.LOW)

def stop_all(pwmA, pwmB):
    if not HAS_GPIO: return
    pwmA.ChangeDutyCycle(0.0)
    pwmB.ChangeDutyCycle(0.0)
    # coast
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.LOW)

def main():
    global duty
    if not HAS_GPIO:
        print("Jetson.GPIO not available; dry-run mode.")
    else:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # Direction pins first
        for p in (IN1, IN2, IN3, IN4):
            GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

        # ---- NVIDIA workaround: setup each PWM pin individually BEFORE creating PWM ----
        GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
        pwmA = GPIO.PWM(ENA, PWM_HZ)
        pwmA.start(0.0)

        GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)
        pwmB = GPIO.PWM(ENB, PWM_HZ)
        pwmB.start(0.0)
        # ------------------------------------------------------------------------------

    print("""
W/S = forward/back
A/D = in-place turn left/right
Q/E = duty -5% / +5%   (current duty affects speed step)
Space = stop (coast)
X = brake (all low)
Esc or Ctrl-C = quit
""")

    # current command (left/right in [-1..1])
    left = 0.0
    right = 0.0

    try:
        while True:
            ch = kbhit()
            if ch in ('\x1b', '\x03'):  # Esc or Ctrl-C
                break
            elif ch in ('w','W'):
                left  = min(1.0, left  + duty/100.0)
                right = min(1.0, right + duty/100.0)
            elif ch in ('s','S'):
                left  = max(-1.0, left  - duty/100.0)
                right = max(-1.0, right - duty/100.0)
            elif ch in ('a','A'):
                # turn left in place: left backward, right forward
                left  = max(-1.0, left  - duty/100.0)
                right = min( 1.0, right + duty/100.0)
            elif ch in ('d','D'):
                # turn right in place: left forward, right backward
                left  = min( 1.0, left  + duty/100.0)
                right = max(-1.0, right - duty/100.0)
            elif ch in ('q','Q'):
                duty = max(5.0, duty - 5.0)
                print(f"duty={duty:.0f}%")
                # don’t change left/right immediately; next key will apply
            elif ch in ('e','E'):
                duty = min(100.0, duty + 5.0)
                print(f"duty={duty:.0f}%")
            elif ch == ' ':
                left = 0.0; right = 0.0
                if HAS_GPIO: stop_all(pwmA, pwmB)
                print("STOP (coast)")
            elif ch in ('x','X'):
                left = 0.0; right = 0.0
                if HAS_GPIO: brake()
                if HAS_GPIO:
                    pwmA.ChangeDutyCycle(0.0); pwmB.ChangeDutyCycle(0.0)
                print("BRAKE (both low)")
            else:
                # ignore unknown keys
                continue

            # apply outputs
            if HAS_GPIO:
                set_side(left,  IN1, IN2, pwmA)
                set_side(right, IN3, IN4, pwmB)

            print(f"L={left:+.2f} R={right:+.2f}  duty={duty:.0f}%")

    except KeyboardInterrupt:
        pass
    finally:
        if HAS_GPIO:
            try:
                stop_all(pwmA, pwmB)
                pwmA.stop(); pwmB.stop()
                for p in (IN1, IN2, IN3, IN4, ENA, ENB):
                    GPIO.output(p, GPIO.LOW)
            finally:
                GPIO.cleanup()

if __name__ == "__main__":
    main()
