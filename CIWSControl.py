# !/usr/bin/python3
# CIWS_Main.py created by alfon at 5/23/2025
"""
Project: CIWS - Autonomous Mobile NErf Gun Turret
Authors: Alfonso Sciacchitano, Matt Gregory, Dylan Bruns, Kristen Tsolis, et. al.
Start Date: 5/23/25
Description: This project enables control of the mobile nerf gun turret system using a Raspberry Pi 5.
Tasks:
- Navigates using Lidar, and possibly IMU.
- Detects and classifies targets using computer vision. OpenCV and Yolo are used.
- Estimates target range via a sonar sensor.
- Fires a servo-actuated Nerf gun at targets classified as threats.

This main script is used to coordinate the overall control of separate modules for each task.
"""

# Python Libraries
import time

# Python Subsystems
from Sensors.navigation import NavigationSystem
from Sensors.camera import VisionSystem
from Sensors.sonar import SonarSystem
from Actuators.trigger import TriggerControl
from Actuators.tracking import YawControl
from Actuators.tracking import PitchControl

# Python Functions
# Define Functions Here

class CIWSControl:
    def __init__(self):
        self.sensor = NavigationSystem()
        self.camera = VisionSystem()
        self.sonar = SonarSystem()
        self.trigger = TriggerControl()
        self.yaw = YawControl()
        self.pitch = PitchControl(launch_speed=2.0)
        self.running = True
        print("CIWS enabled.")

    def main_loop(self):
        while self.running:
            # Get navigation data
            nav_data = self.sensor.get_nav_data()
            print(nav_data)
            # Get camera data
            frame = self.camera.get_frame()

            # Check for targets
            targets = self.camera.get_targets()

            # Engagement logic
            for target in targets:
                distance = self.sonar.get_distance()
                if target["label"] == "enemy" and distance < 5.0:  # assumed name and distance

                    # Aim at target
                    relative_angle = self.camera.get_relative_azimuth(target["bbox"])
                    self.yaw.set_azimuth(relative_angle)
                    self.pitch.set_pitch_from_range(distance)
                    self.trigger.shoot()

    def get_camera_frame(self):
        return None  # Add in the camera logic here

    def stop(self):
        self.trigger.cleanup()  # Stops the PWM command within the trigger control
        self.running = False  # turn off
        print("CIWS Disabled")


if __name__ == '__main__':
    controller = CIWSControl()
    try:
        controller.main_loop()
    except KeyboardInterrupt:
        controller.stop()
        print("Command interrupted.")

# pass
