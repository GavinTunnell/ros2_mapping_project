#!/usr/bin/env python3
import sys, termios, tty, time
import smbus

# ==== USER SETTINGS (edit if needed) ====
BUS_NUM      = 1          # i2c-1
ADDR_ENA     = 0x41       # PCA driving EnA, In1, In2  (RIGHT side by default)
ADDR_ENB     = 0x60       # PCA driving EnB, In3, In4  (LEFT side by default)
ENA_CH       = 0          # EnA PWM channel on ADDR_ENA
IN1_CH       = 1          # In1 digital channel on ADDR_ENA
IN2_CH       = 2          # In2 digital channel on ADDR_ENA
ENB_CH       = 0          # EnB PWM channel on ADDR_ENB
IN3_CH       = 1          # In3 digital channel on ADDR_ENB
IN4_CH       = 2          # In4 digital channel on ADDR_ENB

PWM_FREQ_HZ  = 300.0      # PCA9685 PWM frequency
START_DUTY   = 0.90       # 30% starting duty
STEP_DUTY    = 0.05       # +/-5% per tap
MIN_DUTY     = 0.10       # don't go below 10% (helps overcome stiction)
MAX_DUTY     = 1.00

# If your wheels spin the wrong way, toggle these at runtime with 'R'/'L'
INVERT_RIGHT = True
INVERT_LEFT  = False
# ========================================

# PCA9685 registers
_MODE1      = 0x00
_MODE2      = 0x01
_LED0_ON_L  = 0x06
_PRESCALE   = 0xFE

_RESTART = 1 << 7
_SLEEP   = 1 << 4
_AI      = 1 << 5
_OUTDRV  = 1 << 2  # totem-pole output stage

def _clip(v, lo, hi): return hi if v > hi else lo if v < lo else v

class PCA9685:
    def __init__(self, bus, addr, freq_hz=300.0):
        self.bus = bus
        self.addr = addr
        # sleep + auto-increment to set prescale
        self._w8(_MODE1, _SLEEP | _AI)
        self._w8(_MODE2, _OUTDRV)
        time.sleep(0.005)
        self.set_pwm_freq(freq_hz)
        # wake + restart
        old = self._r8(_MODE1)
        self._w8(_MODE1, (old & ~_SLEEP) | _RESTART | _AI)
        time.sleep(0.005)

    def set_pwm_freq(self, f_hz: float):
        f_hz = float(f_hz)
        prescale = int(round(25_000_000.0 / (4096.0 * f_hz) - 1.0))
        prescale = _clip(prescale, 3, 255)
        old = self._r8(_MODE1)
        self._w8(_MODE1, (old | _SLEEP) & 0x7F)
        self._w8(_PRESCALE, prescale)
        self._w8(_MODE1, old | _AI)
        time.sleep(0.005)
        self._w8(_MODE1, old | _AI | _RESTART)

    def set_off(self, ch: int):
        base = _LED0_ON_L + 4*ch
        # full OFF (OFF_H bit4 = 1)
        self.bus.write_i2c_block_data(self.addr, base, [0x00,0x00,0x00,0x10])

    def set_on(self, ch: int):
        base = _LED0_ON_L + 4*ch
        # full ON (ON_H bit4 = 1)
        self.bus.write_i2c_block_data(self.addr, base, [0x00,0x10,0x00,0x00])

    def set_pwm(self, ch: int, duty01: float):
        duty01 = _clip(duty01, 0.0, 1.0)
        if duty01 <= 0.0: self.set_off(ch); return
        if duty01 >= 1.0: self.set_on(ch);  return
        off = int(round(duty01 * 4095.0))
        base = _LED0_ON_L + 4*ch
        self.bus.write_i2c_block_data(self.addr, base, [0x00,0x00, off & 0xFF, (off>>8) & 0x0F])

    def _w8(self, reg, val): self.bus.write_byte_data(self.addr, reg, val & 0xFF)
    def _r8(self, reg):       return self.bus.read_byte_data(self.addr, reg) & 0xFF


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def print_help(duty, invR, invL):
    print(
f"""
W/A/S/D to move | Space=Coast | B=Brake | +/- duty | R/L invert side | Q quit
Duty: {int(duty*100)}%   Invert Right: {invR}   Invert Left: {invL}
Right = EnA @ 0x{ADDR_ENA:02X} (ENA={ENA_CH}, IN1={IN1_CH}, IN2={IN2_CH})
Left  = EnB @ 0x{ADDR_ENB:02X} (ENB={ENB_CH}, IN3={IN3_CH}, IN4={IN4_CH})
"""
    )

def main():
    bus = smbus.SMBus(BUS_NUM)
    pcaA = PCA9685(bus, ADDR_ENA, PWM_FREQ_HZ)  # EnA, In1, In2 (RIGHT)
    pcaB = PCA9685(bus, ADDR_ENB, PWM_FREQ_HZ)  # EnB, In3, In4 (LEFT)

    # Start safe: everything OFF
    for ch in (ENA_CH, IN1_CH, IN2_CH): pcaA.set_off(ch)
    for ch in (ENB_CH, IN3_CH, IN4_CH): pcaB.set_off(ch)

    duty = START_DUTY
    invR = INVERT_RIGHT
    invL = INVERT_LEFT

    def right_forward():
        fwd = not invR
        if fwd:
            pcaA.set_on(IN1_CH); pcaA.set_off(IN2_CH)
        else:
            pcaA.set_off(IN1_CH); pcaA.set_on(IN2_CH)

    def right_reverse():
        fwd = not invR
        if fwd:
            pcaA.set_off(IN1_CH); pcaA.set_on(IN2_CH)
        else:
            pcaA.set_on(IN1_CH);  pcaA.set_off(IN2_CH)

    def right_coast():
        pcaA.set_off(IN1_CH); pcaA.set_off(IN2_CH)

    def right_brake():
        pcaA.set_on(IN1_CH);  pcaA.set_on(IN2_CH)

    def left_forward():
        fwd = not invL
        if fwd:
            pcaB.set_on(IN3_CH); pcaB.set_off(IN4_CH)
        else:
            pcaB.set_off(IN3_CH); pcaB.set_on(IN4_CH)

    def left_reverse():
        fwd = not invL
        if fwd:
            pcaB.set_off(IN3_CH); pcaB.set_on(IN4_CH)
        else:
            pcaB.set_on(IN3_CH);  pcaB.set_off(IN4_CH)

    def left_coast():
        pcaB.set_off(IN3_CH); pcaB.set_off(IN4_CH)

    def left_brake():
        pcaB.set_on(IN3_CH);  pcaB.set_on(IN4_CH)

    def enable_both(d):
        if d <= 0.0:
            pcaA.set_off(ENA_CH); pcaB.set_off(ENB_CH)
        else:
            pcaA.set_pwm(ENA_CH, d)
            pcaB.set_pwm(ENB_CH, d)

    print_help(duty, invR, invL)

    try:
        while True:
            c = getch()
            if c in ('q', 'Q'):
                break
            elif c in ('+', '='):
                duty = _clip(duty + STEP_DUTY, MIN_DUTY, MAX_DUTY)
                print_help(duty, invR, invL)
            elif c in ('-', '_'):
                duty = _clip(duty - STEP_DUTY, MIN_DUTY, MAX_DUTY)
                print_help(duty, invR, invL)
            elif c in ('r', 'R'):
                invR = not invR
                print_help(duty, invR, invL)
            elif c in ('l', 'L'):
                invL = not invL
                print_help(duty, invR, invL)

            elif c in ('w', 'W'):
                # forward both
                right_forward(); left_forward(); enable_both(duty)
            elif c in ('s', 'S'):
                # reverse both
                right_reverse(); left_reverse(); enable_both(duty)
            elif c in ('a', 'A'):
                # spin left: right fwd, left rev
                right_forward(); left_reverse(); enable_both(duty)
            elif c in ('d', 'D'):
                # spin right: right rev, left fwd
                right_reverse(); left_forward(); enable_both(duty)
            elif c == ' ':
                # coast stop (all IN low, EN off)
                enable_both(0.0); right_coast(); left_coast()
            elif c in ('b', 'B'):
                # brake stop (both IN high, EN off)
                enable_both(0.0); right_brake(); left_brake()
            else:
                # ignore other keys
                pass
    finally:
        # Safe shutdown
        try:
            enable_both(0.0)
            right_coast(); left_coast()
            bus.close()
        except Exception:
            pass
        print("\nExited. Motors OFF.")

if __name__ == "__main__":
    main()
