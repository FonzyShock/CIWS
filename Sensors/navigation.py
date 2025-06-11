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
from math import floor

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
    TIMEOUT = 15  # seconds

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

        # Handle Ctrl+C
        signal.signal(signal.SIGINT, self._cleanup)

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

    def _set_speed(self, pwm: PWMOut, pct: float):
        pwm.duty_cycle = int((pct / 100.0) * 0xFFFF)

    def backward(self, speed=50):
        self.in1.value, self.in2.value = False, True
        self.in3.value, self.in4.value = False, True
        self._set_speed(self.pwmA, speed)
        self._set_speed(self.pwmB, speed)

    def forward(self, speed=50):
        self.in1.value, self.in2.value = True, False
        self.in3.value, self.in4.value = True, False
        self._set_speed(self.pwmA, speed)
        self._set_speed(self.pwmB, speed)

    def turn_left(self, speed=75):
        self.in1.value, self.in2.value = False, True
        self.in3.value, self.in4.value = True, False
        self._set_speed(self.pwmA, speed)
        self._set_speed(self.pwmB, speed)

    def turn_right(self, speed=75):
        self.in1.value, self.in2.value = True, False
        self.in3.value, self.in4.value = False, True
        self._set_speed(self.pwmA, speed)
        self._set_speed(self.pwmB, speed)

    def stop(self):
        self.pwmA.duty_cycle = 0
        self.pwmB.duty_cycle = 0


#  NavigationSystem
class NavigationSystem:
    """Navigation with modes: autonomous, manual, off."""
    def __init__(self):
        # Graceful shutdown on Ctrl+C
        signal.signal(signal.SIGINT, lambda *_: self.shutdown())

        self.mode = self.select_mode()

        if self.mode == 'autonomous':
            #initialize LiDAR for use
            self.lidar = LidarSensor()

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
            print(f"Scan rate set to {self.scan_hz} Hz ({self.scan_period:.3f}s period)")

            # Obstacle threshold (mm)
            self.obstacle_threshold = 500
            # Front sector ±15°
            self.front_sector = set(range(345, 360)) | set(range(0, 15))

            # enable motors for use
            self.motor = MotorController()
            print("Navigation System – Autonomous mode initialized.")
            self.run_autonomous()
        elif self.mode == 'manual':
            self.motor = MotorController()
            print("Navigation System – Manual mode Initialized.")
            self.run_manual()
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

    def control_step(self):
        """Perform one scan and drive/avoid obstacles."""
        d = self.lidar.next_scan() # get distances to all surrounding objects
        # Find minimum distance in front sector
        front = [d[i] for i in self.front_sector if d[i] > 0]
        min_front = min(front) if front else float('inf')

        if min_front > self.obstacle_threshold:
            self.motor.forward()
        else:
            # Compare left (60–120°) vs right (240–300°)
            left  = [d[i] for i in range(60, 120) if d[i] > 0]
            right = [d[i] for i in range(240, 300) if d[i] > 0]
            if min(left or [0]) > min(right or [0]):
                self.motor.turn_left()
            else:
                self.motor.turn_right()

    def run_autonomous(self):
        print("\nStarting obstacle‐avoidance loop. Ctrl+C to exit.")
        while True:
            start = time.time()
            self.control_step()
            # Throttle to desired scan rate
            elapsed = time.time() - start
            if elapsed < self.scan_period:
                time.sleep(self.scan_period - elapsed)

    def run_manual(self):
        print("\nEntering manual mode.")
        print("Use keys for chassis commands:")
        print("  w=forward, s=backward, a=left, d=right, space=stop, q=quit")
        while True:
            raw = input("> ")
            cmd = raw.strip().lower()
            if raw == ' ':
                self.motor.stop()
                print("Stopped")
            elif cmd == 'w':
                self.motor.forward()
                print("Moving forward")
            elif cmd == 'a':
                self.motor.turn_left()
                print("Turning left")
            elif cmd == 'd':
                self.motor.turn_right()
                print("Turning right")
            elif cmd == 's':
                self.motor.backward()
                print("Moving backward")
            elif cmd == 'q':
                self.motor.stop()
                print("Exiting manual mode.")
                break
            else:
                print("Unknown. Use w/a/d/s/space/q.")

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

