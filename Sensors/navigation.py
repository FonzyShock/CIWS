#!/usr/bin/env python3
# navigation.py  created by alfon at 5/23/2025
# Extended  for LiDAR integration on Raspberry Pi 5

import sys
from math import floor
import RPi.GPIO as GPIO
from adafruit_rplidar import RPLidar
import board
from digitalio import DigitalInOut
import signal

# ==== LiDAR helper class ====
class LidarSensor:
    """
    Performs a single 360° scan with RPLidar and returns
    a 360-element distance list (one per degree).
    """
    PORT_NAME = '/dev/ttyUSB0'
    TIMEOUT   = 30    # seconds

    def __init__(self):

        try:
            # Pass motor_pin=None if your model doesn't need it
            self.lidar = RPLidar(None, self.PORT_NAME, timeout=self.TIMEOUT)
        except Exception as e:
            print(f"LiDAR init error unable to connect to LiDAR on{self.PORT_NAME}: {e}", file=sys.stderr)
            sys.exit(1)

        # Ensure a clean shutdown on Ctrl+C
        signal.signal(signal.SIGINT, self._cleanup_and_exit)

    def scan_once(self):
        """Perform one full rotation and return distances[0..359]."""
        distances = [0] * 360
        try:
            # iter_scans yields list of (quality, angle, distance)
            for scan in self.lidar.iter_scans(max_buf_meas=360):
                # fill our buffer
                for _, angle, dist in scan:
                    idx = min(359, floor(angle))
                    distances[idx] = dist
                break  # one rotation
        except Exception as e:
            print(f"LiDAR scan error: {e}", file=sys.stderr)
        finally:
            # stop motor and disconnect to free the port
            self.lidar.stop()
            self.lidar.disconnect()

        return distances

    def _cleanup_and_exit(self, sig, frame):
        """Handle Ctrl+C anywhere in the app."""
        try:
            self.lidar.stop()
            self.lidar.disconnect()
        except:
            pass
        GPIO.cleanup()
        sys.exit(0)

# ---- Differential-Drive Motor Controller ----
class MotorController:
    """
    Controls two DC motors via L298N.
    Left motor:  in1/in2/enA
    Right motor: in3/in4/enB
    """
    def __init__(self,
                 in1=17, in2=27, enA=22,
                 in3=10, in4=9,  enB=11):
        GPIO.setmode(GPIO.BCM)
        pins = [in1, in2, enA, in3, in4, enB]
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        self.in1, self.in2, self.enA = in1, in2, enA
        self.in3, self.in4, self.enB = in3, in4, enB

        self.pwmA = GPIO.PWM(self.enA, 100)
        self.pwmB = GPIO.PWM(self.enB, 100)
        self.pwmA.start(0)
        self.pwmB.start(0)

    def forward(self, speed=50):
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)
        self.pwmA.ChangeDutyCycle(speed)
        self.pwmB.ChangeDutyCycle(speed)

    def turn_left(self, speed=75):
        # Left wheel reverse, right wheel forward
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)
        self.pwmA.ChangeDutyCycle(speed)
        self.pwmB.ChangeDutyCycle(speed)

    def turn_right(self, speed=75):
        # Left wheel forward, right wheel reverse
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)
        self.pwmA.ChangeDutyCycle(speed)
        self.pwmB.ChangeDutyCycle(speed)

    def stop(self):
        self.pwmA.ChangeDutyCycle(0)
        self.pwmB.ChangeDutyCycle(0)



# ==== Main NavigationSystem ====
class NavigationSystem:
    def __init__(self):
        """
        Initialize all navigation sensors:
          • LiDAR
          • IMU (TBD)
          • Encoders (TBD)
        """
        self.lidar = LidarSensor()
        self.motor = MotorController()
        # distance (mm) below which an obstacle triggers avoidance
        self.obstacle_threshold = 500
        # how many degrees on either side of front to consider “front”
        self.front_sector = list(range(350, 360)) + list(range(0, 10))
        print("Navigation System - Initialized.")

    def get_nav_data(self):
        """
        Polls all sensors and returns a unified state dict:
            {
                'x':     float,   # meters
                'y':     float,   # meters
                'theta': float,   # radians
                'lidar': [int],   # 360 distance readings (mm)
            }
        """
        # Placeholder pose data (replace with real IMU/GPS/encoder fusion)
        pose = {
            'x':     0.0,
            'y':     0.0,
            'theta': 0.0
        }

        # Acquire one LiDAR sweep
        lidar_readings = self.lidar.scan_once()

        # Merge into single dict
        nav_data = {**pose, 'lidar': lidar_readings}
        return nav_data

    def control_step(self):
        """Read LiDAR, then either go forward or turn away from obstacle."""
        distances = self.lidar.scan_once()
        # minimum distance in front sector
        front_dists = [distances[i] for i in self.front_sector if distances[i] > 0]
        min_front = min(front_dists) if front_dists else float('inf')

        if min_front > self.obstacle_threshold:
            self.motor.forward()
        else:
            # decide turn direction by comparing left vs right clearance
            left_sector = range(60, 120)
            right_sector = range(240, 300)
            left_min = min([distances[i] for i in left_sector if distances[i] > 0] or [0])
            right_min = min([distances[i] for i in right_sector if distances[i] > 0] or [0])
            if left_min > right_min:
                self.motor.turn_left()
            else:
                self.motor.turn_right()

    def run(self):
        """Continuously execute the control loop until interrupted."""
        signal.signal(signal.SIGINT, lambda *_: self.shutdown())
        print("Starting obstacle-avoidance loop. Ctrl+C to exit.")
        try:
            while True:
                self.control_step()
        except Exception as e:
            print(f"Runtime error: {e}", file=sys.stderr)
        finally:
            self.shutdown()

    def shutdown(self):
        """Stop motors, cleanup GPIO, and exit."""
        print("Shutting down: stopping motors and cleaning up GPIO.")
        self.motor.stop()
        GPIO.cleanup()
        sys.exit(0)


# ==== Example usage ====
if __name__ == '__main__':
    nav = NavigationSystem()
    data = nav.get_nav_data()
    print("Pose:", data['x'], data['y'], data['theta'])
    print("LiDAR[0°..359°]:", data['lidar'])
