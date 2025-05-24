# !/usr/bin/python3
# trigger.py created by alfon at 5/23/2025

"""
Module: servo.py
Subsystem: Actuation / Nerf Gun Firing

Description:
    Provides an interface for triggering a servo motor to fire a Nerf dart.
    The TriggerSystem class encapsulates GPIO PWM logic and exposes a simple
    .shoot() method that can be used by the robot control system.

Usage:
    from actuators.servo import TriggerSystem
    Trigger = TriggerSystem()
    Trigger.shoot()
"""
# Python Libraries
# import RPi.GPIO as GPIO
# import time

# Python Functions



class TriggerControl:
    def __init__(self):
        """
        Initializes the GPIO pin and PWM signal for the servo.
        """
        # Setup:
        # self.SERVO_PIN = 18
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.SERVO_PIN, GPIO.OUT)
        # self.pwm = GPIO.PWM(self.SERVO_PIN, 50)  # 50 Hz
        # self.pwm.start(0)
        print("Trigger Control- Initialized.")

    def shoot(self):
        """
        Activates the servo motor to fire the Nerf gun.
        """
        print("TriggerControl - Firing!")  # for debugging

        # Example for a short pulse:
        # self.pwm.ChangeDutyCycle(7.5)  # Move to fire position
        # time.sleep(0.5)
        # self.pwm.ChangeDutyCycle(2.5)  # Return to rest
        # time.sleep(0.5)

    @staticmethod
    def cleanup():
        """
        Releases the GPIO resources.
        """
        print("TriggerControl- Cleanup.")
        # self.pwm.stop()
        # GPIO.cleanup()

