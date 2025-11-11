#!/usr/bin/env python3
import sys, termios, tty, select, time
import Jetson.GPIO as GPIO

# --- PIN MAP (BOARD numbering) ---
ENA = 32   # PWM A
ENB = 15   # PWM B
IN1 = 23   # B side
IN2 = 19
IN3 = 21   # A side
IN4 = 18

PWM_FREQ = 400  # Hz

def getch(timeout=0.05):
    """Read 1 char with timeout (non-blocking-ish)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    ch = None
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    for p in (IN1, IN2, IN3, IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)
    pa = GPIO.PWM(ENA, PWM_FREQ)
    pb = GPIO.PWM(ENB, PWM_FREQ)
    pa.start(0)
    pb.start(0)
    return pa, pb

def all_low():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def drive_forward():
    # A side forward: IN4=HIGH, IN2=LOW
    # B side forward: IN3=HIGH, IN1=LOW
    GPIO.output(IN1, GPIO.HIGH);  GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH);  GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN1, GPIO.LOW)

def drive_backward():
    GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW);  GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN4, GPIO.HIGH);  GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH);  GPIO.output(IN1, GPIO.HIGH)

def drive_left():
    # Spin left: left motor back, right motor forward
    GPIO.output(IN4, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)  # A backward
    GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN1, GPIO.LOW)   # B forward

def drive_right():
    GPIO.output(IN4, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)   # A forward
    GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN1, GPIO.HIGH)  # B backward

def stop():
    all_low()

def help_text(speed):
    print(
        "\nManual GPIO drive (WASD / IJKL style):\n"
        "  i : forward        , : backward\n"
        "  j : left           l : right\n"
        "  k : stop\n"
        "  + : speed up       - : slow down\n"
        "  q : quit\n"
        f"Current speed: {speed:.0f}%\n"
    )

def main():
    pa, pb = setup()
    duty = 40.0  # percent
    help_text(duty)
    try:
        while True:
            ch = getch(0.1)
            if ch is None:
                continue

            if ch in ('q', 'Q'):
                stop()
                break
            elif ch in ('i', 'I', 'w', 'W'):
                drive_forward()
            elif ch in (',', '<', 's', 'S', 'k', 'K'):  # ',' like teleop for back; also 's'
                drive_backward()
            elif ch in ('j', 'J', 'a', 'A'):
                drive_left()
            elif ch in ('l', 'L', 'd', 'D'):
                drive_right()
            elif ch in (' ', 'x', 'X'):
                stop()
            elif ch in ('+', '='):
                duty = min(100.0, duty + 10.0)
                print(f"Speed: {duty:.0f}%")
            elif ch in ('-', '_'):
                duty = max(0.0, duty - 10.0)
                print(f"Speed: {duty:.0f}%")
            elif ch in ('h', 'H', '?'):
                help_text(duty)

            # Apply PWM duty every loop
            pa.ChangeDutyCycle(duty)
            pb.ChangeDutyCycle(duty)
    finally:
        try:
            stop()
            pa.stop(); pb.stop()
        except Exception:
            pass
        GPIO.cleanup()

if __name__ == "__main__":
    main()
