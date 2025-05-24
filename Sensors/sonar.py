# !/usr/bin/python3
# sonar.py created by alfon at 5/23/2025
"""
Module: sonar.py
Subsystem: Sonar Distance Sensing

Description:
    Used to measure the distance to targets following a positive target classification.
    The SonarSystem class handles trigger-echo timing for the HC-SR04 and converts the signal
    into distance measurements in meters.

    The SonarSystem class handles ..... for the Cone and converts the signal into distance measurements
    in meters.

Usage:
    from Sensors.sonar import SonarSystem
    sonar = SonarSystem()
    distance = sonar.get_distance()
"""

# Python Libraries
# import RPi.GPIO as GPIO
import time


# Python Functions
# Define Functions Here

class SonarSystem:
    def __init__(self):
        """
        Initialize GPIO pins for sonar sensor.
        Replace with actual GPIO pin setup if using HC-SR04.
        """
        # Example GPIO setup:
        # self.TRIG = 23
        # self.ECHO = 24
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.TRIG, GPIO.OUT)
        # GPIO.setup(self.ECHO, GPIO.IN)
        print("Sonar System - Initialized.")

    def get_distance(self):
        """
        Measures distance to the nearest object using sonar.

        Returns:
            float: distance in meters
        """
        # Example HC-SR04 measurement logic:
        # GPIO.output(self.TRIG, True)
        # time.sleep(0.00001)
        # GPIO.output(self.TRIG, False)

        # while GPIO.input(self.ECHO) == 0:
        #     pulse_start = time.time()
        # while GPIO.input(self.ECHO) == 1:
        #     pulse_end = time.time()

        # pulse_duration = pulse_end - pulse_start
        # distance = (pulse_duration * 343.0) / 2.0  # m/s * sec = meters (div 2 round trip)
        distance = 1.5
        print("Distance to target (m):" + str(distance))
        return distance  # Simulated distance in meters
