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

def get_char(timeout=0.1):
    """Non-blocking single-key reader for manual input."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        return sys.stdin.read(1) if rlist else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

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

        if self.mode == "manual":
            self._manual_thread = threading.Thread(target=self._manual_control, daemon=True)
            self._manual_thread.start()

    def aim(self, center_x, center_y):
        """For auto mode only — convert pixel coordinates to servo angles."""
        if self.mode != "auto":
            return
        with self._lock:
            self.angle_x = clamp(int((center_x / 640.0) * 180))
            self.angle_z = clamp(int((center_y / 480.0) * 180))
            self.servo_x.angle = self.angle_x
            self.servo_z.angle = self.angle_z
            print(f"[Trigger] Auto-aiming to X={self.angle_x}, Z={self.angle_z}")

    def shoot(self):
        with self._lock:
            print("[Trigger] Firing!")
            self.servo_TRIGGER.angle = TRIGGER_FIRE_ANGLE
            time.sleep(1)
            self.servo_TRIGGER.angle = TRIGGER_RETRACT_ANGLE
            time.sleep(1)
            print("[Trigger] Trigger reset.")

    def _manual_control(self):
        print("Manual controls: i=up, k=down, j=left, l=right, space=fire, q=quit")
        try:
            while self.running:
                ch = get_char()
                self.handle_key(ch)
                time.sleep(0.05)
        except Exception as e:
            print(f"[Trigger] Manual thread error: {e}")
            self.running = False

    def handle_key(self, key):
        if self.mode != "manual":
            return
        with self._lock:
            if key == 'i':
                self.angle_z = clamp(self.angle_z - ANGLE_STEP)
            elif key == 'k':
                self.angle_z = clamp(self.angle_z + ANGLE_STEP)
            elif key == 'j':
                self.angle_x = clamp(self.angle_x - ANGLE_STEP)
            elif key == 'l':
                self.angle_x = clamp(self.angle_x + ANGLE_STEP)
            elif key == ' ':
                self.shoot()
            elif key == 'q':
                self.running = False
            self.servo_x.angle = self.angle_x
            self.servo_z.angle = self.angle_z
    # def _manual_control(self):
    #     print("Manual controls: i=up, k=down, j=left, l=right, space=fire, q=quit")
    #     try:
    #         while self.running:
    #             ch = get_char()
    #             with self._lock:
    #                 if ch == 'i':
    #                     self.angle_z = clamp(self.angle_z - ANGLE_STEP)
    #                 elif ch == 'k':
    #                     self.angle_z = clamp(self.angle_z + ANGLE_STEP)
    #                 elif ch == 'j':
    #                     self.angle_x = clamp(self.angle_x - ANGLE_STEP)
    #                 elif ch == 'l':
    #                     self.angle_x = clamp(self.angle_x + ANGLE_STEP)
    #                 elif ch == ' ':
    #                     self.shoot()
    #                 elif ch == 'q':
    #                     self.running = False
    #             self.servo_x.angle = self.angle_x
    #             self.servo_z.angle = self.angle_z
    #             time.sleep(0.05)
    #     except Exception as e:
    #         print(f"[Trigger] Manual thread error: {e}")
    #         self.running = False

    def cleanup(self):
        self.running = False
        self.pwm_x.deinit()
        self.pwm_z.deinit()
        self.pwm_TRIGGER.deinit()
        print("[Trigger] Cleanup complete.")

# === Test Mode ===
if __name__ == "__main__":
    trig = TriggerControl(mode="manual")
    try:
        while trig.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        trig.cleanup()
