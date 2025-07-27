#!/usr/bin/python3
# trigger.py

"""
Terminal-based servo control for Nerf dart trigger and X/Z servos,
using termios for single-key input (i/j/k/l = up/left/down/right,
SPACE = fire, q = quit).
"""

import board
import pwmio
from adafruit_motor import servo
import sys
import termios
import tty
import select
import time

# --- Configuration ---
SERVO_X_PIN    = board.D17   # X-axis servo
SERVO_Z_PIN    = board.D18   # Z-axis servo
SERVO_DART_PIN = board.D24   # dart-firing servo

# Key mappings
KEY_UP     = 'i'
KEY_DOWN   = 'k'
KEY_LEFT   = 'j'
KEY_RIGHT  = 'l'
KEY_FIRE   = ' '   # space
KEY_QUIT   = 'q'

# Dart angles (0–180°)
DART_FIRE_ANGLE    = 180
DART_RETRACT_ANGLE = 0

# Movement step per keypress
ANGLE_STEP = 2


class TriggerSystem:
    """Encapsulates the dart-firing servo."""
    def __init__(self, pin):
        self._pwm = pwmio.PWMOut(pin, frequency=50)
        self.servo = servo.Servo(self._pwm)
        self.servo.angle = DART_RETRACT_ANGLE
        print(f"[Trigger] initialized on {pin}")

    def shoot(self):
        print("[Trigger] firing!")
        self.servo.angle = DART_FIRE_ANGLE
        time.sleep(2)
        self.servo.angle = DART_RETRACT_ANGLE
        time.sleep(0.5)
        print("[Trigger] cycle complete.")

    def cleanup(self):
        self._pwm.deinit()
        print("[Trigger] cleaned up.")


def clamp(x, lo=0, hi=180):
    return max(lo, min(hi, x))


def get_char(timeout=0.1):
    """
    Read a single character from stdin, nonblocking with timeout.
    Returns '' if no key was pressed within timeout.
    """
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    # Initialize servos
    print("Servo test started. Waiting for input...")

    pwm_x = pwmio.PWMOut(SERVO_X_PIN, frequency=100)
    pwm_z = pwmio.PWMOut(SERVO_Z_PIN, frequency=100)
    servo_x = servo.Servo(pwm_x)
    servo_z = servo.Servo(pwm_z)
    trigger = TriggerSystem(SERVO_DART_PIN)

    # Start centered
    servo_x.angle = 90
    servo_z.angle = 90
    print("Controls: i=up, k=down, j=left, l=right, SPACE=fire, q=quit")

    running = True
    try:
        while running:
            ch = get_char()

            if not ch:
                time.sleep(0.05)
                continue

            if ch == KEY_QUIT:
                running = False

            elif ch == KEY_UP:
                servo_z.angle = clamp(servo_z.angle + ANGLE_STEP)
                print(f"[Z] angle → {servo_z.angle}")

            elif ch == KEY_DOWN:
                servo_z.angle = clamp(servo_z.angle - ANGLE_STEP)
                print(f"[Z] angle → {servo_z.angle}")

            elif ch == KEY_LEFT:
                servo_x.angle = clamp(servo_x.angle - ANGLE_STEP)
                print(f"[X] angle → {servo_x.angle}")

            elif ch == KEY_RIGHT:
                servo_x.angle = clamp(servo_x.angle + ANGLE_STEP)
                print(f"[X] angle → {servo_x.angle}")

            elif ch == KEY_FIRE:
                trigger.shoot()
            time.sleep(0.05)  # Prevents 100% CPU usage during idle

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        # Cleanup all hardware
        pwm_x.deinit()
        pwm_z.deinit()
        trigger.cleanup()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
