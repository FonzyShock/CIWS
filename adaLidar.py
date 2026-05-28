#!/usr/bin/env python3
# lidar_scan.py
# Raspberry Pi 5 — Simple 360° LiDAR Scanner
# Adapted from Litter Detection Robot code by Kutluhan Aktar
# Author: Your Name
# Date:   2025-06-09

import sys
import signal
from math import floor

import board
from digitalio import DigitalInOut
from adafruit_rplidar import RPLidar


# ==== Configuration ====
PORT_NAME = '/dev/ttyUSB0'    # USB port to which the LiDAR is connected
TIMEOUT   = 30                # seconds to wait for device response
ANGLE_RES = 360               # degrees of resolution

class LidarScanner:
    """Simple 360° environment scanner using RPLidar."""
    def __init__(self, port=PORT_NAME, timeout=TIMEOUT):
        try:
            self.lidar = RPLidar(None, port, timeout=timeout)
        except Exception as e:
            print(f"ERROR: Unable to connect to LiDAR on {port}: {e}", file=sys.stderr)
            sys.exit(1)
        self.is_scanning = False
        # Prepare an array to hold one distance per degree
        self.distances = [0] * ANGLE_RES

    def _signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully."""
        print("\nSIGINT received: stopping scan and exiting.")
        self.stop()
        sys.exit(0)

    def start(self):
        """Begin continuous scanning; install SIGINT handler."""
        signal.signal(signal.SIGINT, self._signal_handler)
        self.is_scanning = True
        print("LiDAR scan started. Press Ctrl+C to stop.")
        try:
            for scan in self.lidar.iter_scans(max_buf_meas=ANGLE_RES):
                # Reset distance buffer
                self.distances = [0] * ANGLE_RES
                # Fill distances by angle (0–359°)
                for (_, angle, distance) in scan:
                    idx = min(ANGLE_RES - 1, floor(angle))
                    self.distances[idx] = distance
                self.process_scan(self.distances)
        except Exception as e:
            print(f"ERROR during scan: {e}", file=sys.stderr)
        finally:
            self.stop()

    def process_scan(self, distances):
        """
        Callback for each full 360° scan.
        Override or extend this to integrate with your application.
        """
        # Example: print every 30° for brevity
        sampled = {deg: distances[deg] for deg in range(0, ANGLE_RES, 30)}
        print("Sampled distances (every 30°):", sampled)

    def stop(self):
        """Stop the LiDAR motor, disconnect, and clean up."""
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
            except:
                pass
        print("LiDAR stopped and disconnected.")

if __name__ == '__main__':
    scanner = LidarScanner()
    scanner.start()
