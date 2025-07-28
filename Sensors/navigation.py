#!/usr/bin/env python3
# navigation.py
# Created by Alfonso S. on 2025-06-10 (updated 2025-06-11)
# Subsystem: Navigation Sensor & Differential‐Drive Controller

"""
Module: navigation.py
Subsystem: Navigation Sensor & Motor Control

Description:
    • Auto‐detect & continuously scan RPLidar at a user‐configurable rate
    • Two‐motor differential drive via L298N (Blinka digitalio + pwmio)
    • Simple obstacle avoidance: drive forward, turn away

Options:
  1. Autonomous: perform continuous scans at user‑specified rate (1–100 Hz).
  2. Manual: accept simple keyboard commands for driving.
  3. Off: robot remains stationary.

Usage:
    python3 navigation.py
"""

import sys
import signal
import time
import threading
from math import floor
import termios
import tty
import select

# RPLidar
from adafruit_rplidar import RPLidar
import serial.tools.list_ports

# GPIO & PWM (Blinka)
import board
from digitalio import DigitalInOut, Direction
from pwmio import PWMOut


# LidarSensor
class LidarSensor:
    """Auto‐detect RPLidar port and produce continuous 360° scans."""
    TIMEOUT = 30  # seconds

    @staticmethod
    def find_port():
        # Look for known CP210x adapter (VID=0x10C4, PID=0xEA60) if windows
        for port in serial.tools.list_ports.comports():
            if port.vid == 0x10C4 and port.pid == 0xEA60:
                return port.device
        # Fallback to any /dev/ttyUSB*
        for port in serial.tools.list_ports.comports():
            if port.device.startswith("/dev/ttyUSB"):
                return port.device
        raise RuntimeError("RPLidar port not found")

    def __init__(self):
        try:
            self.port = self.find_port()
        except RuntimeError as e:
            print(f"LiDAR error: {e}", file=sys.stderr)
            sys.exit(1)
        print(f"RPLidar detected on {self.port}")

        try:
            self.lidar = RPLidar(None, self.port, timeout=self.TIMEOUT)
        except Exception as e:
            print(f"LiDAR init error: {e}", file=sys.stderr)
            sys.exit(1)

        # Start motor & begin scans
        self.lidar.start_motor()
        self.scan_iter = self.lidar.iter_scans(max_buf_meas=360)



    def next_scan(self):
        """
        Returns: list of 360 distances (mm); zeros if no return.
        """
        distances = [0] * 360
        try:
            scan = next(self.scan_iter)
            for _, angle, dist in scan:
                idx = min(359, floor(angle))
                distances[idx] = dist
        except Exception as e:
            print(f"LiDAR scan error: {e}", file=sys.stderr)
        return distances

    def _cleanup(self, *args):
        """Stop motor & disconnect on program exit."""
        try:
            self.lidar.stop()
            self.lidar.disconnect()
        except:
            pass
        sys.exit(0)


# MotorController
def set_speed(pwm: PWMOut, pct: float):
    pwm.duty_cycle = int((pct / 100.0) * 0xFFFF)

class MotorController:
    """ Differential drive via L298N """
    def __init__(self, freq=1000):
        # Direction pins
        self.in1 = DigitalInOut(board.D26); self.in1.direction = Direction.OUTPUT
        self.in2 = DigitalInOut(board.D13); self.in2.direction = Direction.OUTPUT
        self.in3 = DigitalInOut(board.D21); self.in3.direction = Direction.OUTPUT
        self.in4 = DigitalInOut(board.D27);  self.in4.direction = Direction.OUTPUT
        # PWM pins
        self.pwmA = PWMOut(board.D19, frequency=freq, duty_cycle=0)
        self.pwmB = PWMOut(board.D4, frequency=freq, duty_cycle=0)

        while True:
            try:
                self.forward_speed = float(input("Enter forward speed (25-100%): "))
                if 25 <= self.forward_speed <= 100:
                    break
            except ValueError:
                pass
            print("Invalid input. Enter a number between 25 and 100.")
        while True:
            try:
                self.turn_speed = float(input("Enter turn speed (25-100%): "))
                if 25 <= self.turn_speed <= 100:
                    break
            except ValueError:
                pass
            print("Invalid input. Enter a number between 25 and 100.")
        while True:
            try:
                self.backward_speed = float(input("Enter reverse speed (25-100%): "))
                if 25 <= self.turn_speed <= 100:
                    break
            except ValueError:
                pass
            print("Invalid input. Enter a number between 25 and 100.")

    def backward(self):
        self.in1.value, self.in2.value = False, True
        self.in3.value, self.in4.value = False, True
        set_speed(self.pwmA, self.backward_speed)
        set_speed(self.pwmB, self.backward_speed)

    def forward(self):
        self.in1.value, self.in2.value = True, False
        self.in3.value, self.in4.value = True, False
        set_speed(self.pwmA, self.forward_speed)
        set_speed(self.pwmB, self.forward_speed)

    def turn_left(self):
        self.in1.value, self.in2.value = False, True
        self.in3.value, self.in4.value = True, False
        set_speed(self.pwmA, self.turn_speed)
        set_speed(self.pwmB, self.turn_speed)

    def turn_right(self):
        self.in1.value, self.in2.value = True, False
        self.in3.value, self.in4.value = False, True
        set_speed(self.pwmA, self.turn_speed)
        set_speed(self.pwmB, self.turn_speed)

    def stop(self):
        self.pwmA.duty_cycle = 0
        self.pwmB.duty_cycle = 0


#  NavigationSystem
class NavigationSystem:
    """Navigation with modes: autonomous, manual, off."""
    def __init__(self):
        # # Graceful shutdown on Ctrl+C
        # signal.signal(signal.SIGINT, lambda *_: self.shutdown())

        self.mode = self.select_mode()

        if self.mode == 'autonomous':
            #initialize LiDAR and motor controller
            self.lidar = LidarSensor()
            self.motor = MotorController()

            # Scan rate configuration
            while True:
                try:
                    rate = float(input("Enter LiDAR scan rate (1–100 Hz): "))
                    if 1 <= rate <= 100:
                        self.scan_hz = rate
                        break
                except ValueError:
                    pass
                print("Invalid input. Enter a number between 1 and 100.")
            self.scan_period = 1.0 / self.scan_hz
            print(f"Scan rate set to {self.scan_hz} Hz")

            '''Obstacle thresholds for turning and reverse motion'''
            while True:
                try:
                    turn_threshold = float(input("Enter minimum obstacle turn distance (mm): "))
                    if 100 <= turn_threshold <= 1000:
                        self.turn_threshold = turn_threshold
                        break
                    else:
                        print("Invalid input. Enter a number between 100 and 1000 mm.")
                except ValueError:
                    print("Invalid input. Please enter a valid number.")
            print(f"Turn threshold set to {self.turn_threshold} mm")

            while True:
                try:
                    reverse_threshold = float(input("Enter minimum obstacle reverse distance (mm): "))
                    if  reverse_threshold >= self.turn_threshold:
                        print("Invalid input. Reverse threshold must be less than the turn threshold.")
                    elif reverse_threshold > 100:
                        print("Invalid input. Reverse threshold must be at less than 100 mm.")
                    else:
                        self.reverse_threshold = reverse_threshold
                        break
                except ValueError:
                    pass
                print("Invalid input. Enter a valid number.")
            print(f"Reverse threshold set to {self.reverse_threshold} mm")

            ''' Set the field-of-view'''
            # Front sector ±15°
            self.front_sector = set(range(345, 360)) | set(range(0, 15))

            print("Navigation System – Autonomous mode initialized.")
            print("\nStarting obstacle‐avoidance loop. Ctrl+C to exit.")
            self._autonomous_thread = threading.Thread(target=self._run_autonomous, daemon=True)
            self._autonomous_thread.start()

        elif self.mode == 'manual':
            self.motor = MotorController()
           # self.old_settings = termios.tcgetattr(sys.stdin)
           # tty.setcbreak(sys.stdin.fileno())
            print("Navigation System – Manual mode Initialized.")
            #self._manual_thread = threading.Thread(target=self.run_manual, daemon=True)
            #self._manual_thread.start()
        else:
            print("Navigation is off - Sentry mode Initialized.")

    @staticmethod
    def select_mode():
        print("Select navigation mode:")
        print("  [A]utonomous")
        print("  [M]anual")
        print("  [O]ff")
        while True:
            choice = input("Enter A, M, or O: ").strip().lower()
            if choice in ('a', 'm', 'o'):
                return {'a': 'autonomous', 'm': 'manual', 'o': 'off'}[choice]
            print("Invalid choice. Please enter A, M, or O.")

    def _run_autonomous(self):
        """method is for internal use only (note leading underscore).
        Cannot be used outside the Navigation system method. """
        try:
            while True:  # not self._stop_event.is_set():
                start_time = time.time()

                # --- LiDAR Scan and Preprocessing ---
                distances = self.lidar.next_scan()  # Full 360° LiDAR scan
                front_distances = [distances[i] for i in self.front_sector if distances[i] > 0]
                min_front = min(front_distances) if front_distances else float('inf')

                # --- Obstacle Avoidance Logic ---
                if min_front > self.turn_threshold:
                    # Safe to drive forward
                    self.motor.forward()
                    print('No obstacles detected. Driving forward.')
                elif min_front < self.reverse_threshold:
                    # Too close to an obstacle; reverse
                    self.motor.backward()
                    print('Reversing...')
                else:
                    # Obstacle within caution zone; determine best turn direction
                    left_distances = [distances[i] for i in range(60, 120) if distances[i] > 0]
                    right_distances = [distances[i] for i in range(240, 300) if distances[i] > 0]

                    min_left = min(left_distances) if left_distances else float('inf')
                    min_right = min(right_distances) if right_distances else float('inf')

                    if min_left < min_right:
                        self.motor.turn_right()  # Obstacle closer on the left
                        print('Turning right...')
                    else:
                        self.motor.turn_left()  # Obstacle closer on the right
                        print('Turning left...')

                # --- Loop Timing Control to Match Desired Scan Rate ---
                elapsed_time = time.time() - start_time
                sleep_time = self.scan_period - elapsed_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            # Graceful shutdown on manual interruption
            pass
        finally:
            self.shutdown()

    def handle_key(self, key):
        if self.mode != 'manual' or self.motor is None:
            return

        if key == 'w':
            self.motor.forward()
        elif key == 'a':
            self.motor.turn_left()
        elif key == 'd':
            self.motor.turn_right()
        elif key == 's':
            self.motor.backward()
        else:
            self.motor.stop()
    # def run_manual(self):
    #     print("\nEntering manual mode.")
    #     print("Press keys to drive:")
    #     print("  w=forward, s=backward, a=left, d=right, q=quit")
    #     try:
    #         while True:
    #             ch = None
    #             if select.select([sys.stdin], [], [], 0.1)[0]:
    #                 ch = sys.stdin.read(1)
    #             if ch == 'w':
    #                 self.motor.forward()
    #             elif ch == 'a':
    #                 self.motor.turn_left()
    #             elif ch == 'd':
    #                 self.motor.turn_right()
    #             elif ch == 's':
    #                 self.motor.backward()
    #             elif ch == 'q':
    #                 break
    #             else:
    #                 self.motor.stop()
    #     finally:
    #         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    #         self.motor.stop()
    #         print("Exiting manual mode.")

    def shutdown(self):
        """Stop motors & LiDAR, then exit."""
        print("Shutting down...")
        self.motor.stop()
        try:
            self.lidar.lidar.stop()
            self.lidar.lidar.disconnect()
        except:
            pass
        sys.exit(0)


if __name__ == '__main__':
    nav = NavigationSystem()

