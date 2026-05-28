#!/usr/bin/python3
# CIWSControl.py

"""
Project: CIWS - Autonomous Mobile Nerf Gun Turret
Description: Controls navigation, vision-based target detection, and trigger firing.
Authors: Alfonso S., et al.
"""

import time
import threading
import signal
import sys

# === Subsystem Imports ===
from Sensors.navigation import NavigationSystem
from Sensors.camera import PersonDetector
#from Actuators.trigger import TriggerSystem

class CIWSControl:
    def __init__(self, model_path="/home/CIWS/yolov11n.pt", camera_id=0):
        print("[CIWS] Initializing subsystems...")
        try:
            # Navigation subsystem thread
            self.nav_thread = threading.Thread(target=self.start_navigation, daemon=True)
            self.nav_ready = False
        except ValueError as e:
            print(f"Failed to init Nav System: {e}")
        else:
            print(f"Nav init complete.")

        try:
            # Person detection
            self.detector = PersonDetector(model_path=model_path, camera_id=camera_id)

        except ValueError as e:
            print(f"Failed to init Vision System: {e}")
        else:
            print(f"Vison system init complete.")

        # try:
        #     # Trigger servos for pan/tilt and firing
        #     self.trigger = TriggerSystem()
        # except ValueError as e:
        #     print(f"Failed to init targeting system: {e}")
        # else:
        #     print(f"Targeting system init complete.")
        # finally:
            self.running = True
            print("[CIWS] Initialization complete.")

    def start_navigation(self):
        """Instantiate the navigation system (blocks until shutdown)."""
        self.nav = NavigationSystem()
        self.nav_ready = True

    def _watchdog_loop(self):
        """Monitor subsystem comms. Force safe state if anything goes stale."""
        print("[Watchdog] Started.")
        while self.running:
            time.sleep(0.5)
            stale = self.watchdog.check()
            if stale:
                print(f"[Watchdog] STALE SUBSYSTEMS: {stale} -- forcing safe state.")
                # Stop motors
                try:
                    if self.navigation and self.navigation.motor:
                        self.navigation.motor.stop()
                except Exception:
                    pass
                # Center servos (don't keep aiming with stale data)
                try:
                    if self.trigger:
                        self.trigger.center()
                except Exception:
                    pass
        print("[Watchdog] Stopped.")

    def main_loop(self):
        print("[CIWS] Launching navigation and detection threads...")
        self.nav_thread.start()

        if not self.detector.start():
            raise RuntimeError("[CIWS] Failed to start PersonDetector.")
        self.detector_thread = threading.Thread(target=self.detector.run, daemon=True)
        self.detector.start()

        # Wait for navigation to be ready
        while not self.nav_ready:
            time.sleep(0.1)

        print("[CIWS] Main loop running. Press Ctrl+C to stop.")
        try:
            while self.running:
                # Poll for the latest person detection
                if self.detector.parent_conn.poll(0.1):
                    detection = self.detector.parent_conn.recv()
                    if detection:
                        cx = detection.get("center_x")
                        cy = detection.get("center_y")
                        print(f"[CIWS] Person at ({cx}, {cy})")
                        self.trigger.aim(cx, cy)

                time.sleep(0.01)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        if self.running:
            print("[CIWS] Shutting down...")
            self.running = False

            # Stop detector
            try:
                self.detector.stop()
            except Exception:
                pass

            # Cleanup trigger
            try:
                self.trigger.cleanup()
            except Exception:
                pass

            # Shutdown navigation
            try:
                self.nav.shutdown()
            except Exception:
                pass

            print("[CIWS] All systems offline.")
            sys.exit(0)

if __name__ == "__main__":
    controller = CIWSControl()
    # Redirect Ctrl+C to our stop
    signal.signal(signal.SIGINT, lambda *_: controller.stop())
    controller.main_loop()
