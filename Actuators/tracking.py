# !/usr/bin/python3
# tracking.py created by alfon at 5/23/2025
"""
Module: tracking.py
Subsystem: Turret Yaw and Pitch Control

Description:
    Provides coordinated yaw (azimuth) and pitch (elevation) control for turret aiming.
    YawControl handles horizontal rotation of the turret, and PitchControl computes
    and adjusts the firing angle based on ballistic equations using measured range.

Usage:
    from actuators.tracking import YawControl, PitchControl

    yaw = YawControl()
    pitch = PitchControl(launch_speed=10.0)

    yaw.set_azimuth(desired_heading_deg)
    pitch.set_pitch_from_range(range_to_target_meters)
"""

# Python Libraries
import math
# import RPi.GPIO as GPIO
# import time

# Python Functions
# Define Functions Here


class YawControl:
    def __init__(self):
        """
        Initializes the turret's yaw control hardware.
        """
        # GPIO setup
        # self.PIN = 17
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.PIN, GPIO.OUT)
        # self.pwm = GPIO.PWM(self.PIN, 50)
        # self.pwm.start(0)
        print("Turret Yaw Control - Initialized.")

    def set_azimuth(self, angle_deg):
        """
        Rotates the turret to the specified azimuth angle.
        Args:
            angle_deg (float): desired yaw angle in degrees
        """
        print(f"YawControl - Setting azimuth to {angle_deg:.1f}°")
        # duty = angle_deg / 18 + 2.5
        # self.pwm.ChangeDutyCycle(duty)
        # time.sleep(0.3)


class PitchControl:
    def __init__(self, launch_speed=5.0):
        """
        Initializes pitch control and assumes a launch speed of 5 m/s.

        Args:
            launch_speed (float): dart speed in m/s
        """
        self.v0 = launch_speed
        self.g = 9.81
        print(f"PitchControl - Initialized with launch speed {self.v0} m/s")

    def compute_pitch_angle(self, range_m):
        """
        Computes ideal pitch angle for given range using basic projectile motion.
        Args:
            range_m (float): horizontal range to the target in meters
        Returns:
            float: angle in degrees
        """
        try:
            angle_rad = 0.5 * math.asin((self.g * range_m) / (self.v0 ** 2))
            return math.degrees(angle_rad)
        except ValueError:
            print("Pitch Control - Max range reached!")
            return 45.0  # max range

    def set_pitch_from_range(self, range_m):
        """
        Sets the turret's pitch based on the computed angle from the target range.
        Args:
            range_m (float): distance to the target in meters
        """
        angle = self.compute_pitch_angle(range_m)
        print(f"PitchControl - Setting pitch to {angle:.2f}° for range {range_m} m")
        # duty = angle / 18 + 2.5
        # self.pwm.ChangeDutyCycle(duty)
        # time.sleep(0.3)
