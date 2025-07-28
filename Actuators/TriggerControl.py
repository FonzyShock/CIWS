# !/usr/bin/python3
# TriggerControl.py created by Desktop at 7/27/2025

import board
import pwmio
from adafruit_motor import servo
import time
import threading
import sys
import termios
import tty
import select

# --- Servo Config ---
SERVO_X_PIN    = board.D17
SERVO_Z_PIN    = board.D18
SERVO_TRIGGER_PIN = board.D24

TRIGGER_FIRE_ANGLE    = 180
TRIGGER_RETRACT_ANGLE = 0
ANGLE_STEP = 5

def clamp(x, lo=0, hi=180):
    return max(lo, min(hi, x))

class TriggerControl:
    def __init__(self, mode=None):
        """Initialize servo control; mode can be 'auto' or 'manual'."""
        if mode is None:
            while True:
                user_input = input("Select trigger mode ([A]uto / [M]anual): ").strip().lower()
                if user_input in ['a', 'auto']:
                    self.mode = "auto"
                    break
                elif user_input in ['m', 'manual']:
                    self.mode = "manual"
                    break
                else:
                    print("Invalid input. Please enter 'A' for auto or 'M' for manual.")
        else:
            self.mode = mode

        print(f"[Trigger] Initialized in {self.mode.upper()} mode.")
        self._lock = threading.Lock()
        self.running = True

        # Hardware init
        self.pwm_x = pwmio.PWMOut(SERVO_X_PIN, frequency=50)
        self.pwm_z = pwmio.PWMOut(SERVO_Z_PIN, frequency=50)
        self.pwm_TRIGGER = pwmio.PWMOut(SERVO_TRIGGER_PIN, frequency=50)

        self.servo_x = servo.Servo(self.pwm_x)
        self.servo_z = servo.Servo(self.pwm_z)
        self.servo_TRIGGER = servo.Servo(self.pwm_TRIGGER)

        self.angle_x = 90
        self.angle_z = 90
        self.servo_x.angle = self.angle_x
        self.servo_z.angle = self.angle_z


    def aim(self, center_x, center_y):
        """For auto mode only — convert pixel coordinates to servo angles."""
        if self.mode != "auto":
            return
        with self._lock:
            self.angle_x = clamp(int((center_x / 1920.0) * 180))
            self.angle_z = clamp(int((center_y / 1080.0) * 180))
            self.servo_x.angle = self.angle_x
            self.servo_z.angle = self.angle_z
            print(f"[Trigger] Auto-aiming to X={self.angle_x}, Z={self.angle_z}")

    def shoot_auto(self):
        print("[Trigger] Auto Firing!")
        with self._lock:
            self.servo_TRIGGER.angle = TRIGGER_FIRE_ANGLE
        time.sleep(1)
        with self._lock:
            self.servo_TRIGGER.angle = TRIGGER_RETRACT_ANGLE
        time.sleep(1)
        print("[Trigger] Auto Trigger reset.")

    def shoot_manual(self):
        print("[Trigger] Manual Firing!")
        self.servo_TRIGGER.angle = TRIGGER_FIRE_ANGLE
        time.sleep(1)
        self.servo_TRIGGER.angle = TRIGGER_RETRACT_ANGLE
        time.sleep(1)
        print("[Trigger] Manual Trigger reset.")

    def handle_key(self, key):
        if self.mode != "manual":
            return
        print(f"[Keyboard] Trigger key: {repr(key)}")  # Debug: confirm dispatcher input
        with self._lock:
            if key == 'i':
                self.angle_z = clamp(self.angle_z - ANGLE_STEP)
            elif key == 'k':
                self.angle_z = clamp(self.angle_z + ANGLE_STEP)
            elif key == 'j':
                self.angle_x = clamp(self.angle_x - ANGLE_STEP)
            elif key == 'l':
                self.angle_x = clamp(self.angle_x + ANGLE_STEP)
            elif key in 'f':
                self.shoot_manual()
            elif key == 'q':
                self.running = False
            self.servo_x.angle = self.angle_x
            self.servo_z.angle = self.angle_z

    def cleanup(self):
        self.running = False
        self.pwm_x.deinit()
        self.pwm_z.deinit()
        self.pwm_TRIGGER.deinit()
        print("[Trigger] Cleanup complete.")

# === Test Mode ===
if __name__ == "__main__":
    from KeyboardDispatcher import KeyboardDispatcher

    trig = TriggerControl(mode="manual")


    def trig_handler(key):
        trig.handle_key(key)


    kb = KeyboardDispatcher(trigger_handler=trig_handler)
    kb.start()

    try:
        while trig.running and kb.running:
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        kb.stop()
        trig.cleanup()
