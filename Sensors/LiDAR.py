#!/usr/bin/env python3
# lidar_scan_visual.py
# Raspberry Pi 5 — 360° LiDAR Scanner with Live Point-Cloud Display
# Author: Alfonso S
# Date:   2025-06-09

import sys
import signal
from math import floor, radians, cos, sin
from adafruit_rplidar import RPLidar
import board
from digitalio import DigitalInOut

# Matplotlib for live plotting
import matplotlib.pyplot as plt

# ==== Configuration ====
PORT_NAME = '/dev/ttyUSB0'
TIMEOUT   = 30
ANGLE_RES = 360

class LidarScanner:
    """360° LiDAR scanner with live 2D point-cloud visualization."""
    def __init__(self, port=PORT_NAME, timeout=TIMEOUT):
        # --- Initialize LiDAR ---
        try:
            self.lidar = RPLidar(None, port, timeout=timeout)
        except Exception as e:
            print(f"ERROR: Unable to connect to LiDAR on {port}: {e}", file=sys.stderr)
            sys.exit(1)
        self.is_scanning = False
        self.distances = [0] * ANGLE_RES

        # --- Setup Matplotlib figure/axes ---
        plt.ion()  # interactive mode on
        self.fig, self.ax = plt.subplots(figsize=(6,6))
        self.scatter = self.ax.scatter([], [], s=2)
        self.ax.set_xlim(-6000, 6000)  # adjust to your LiDAR range in mm
        self.ax.set_ylim(-6000, 6000)
        self.ax.set_aspect('equal', 'box')
        self.ax.set_title("LiDAR Point Cloud")
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        plt.show()

    def _signal_handler(self, sig, frame):
        print("\nSIGINT received: stopping scan and exiting.")
        self.stop()
        sys.exit(0)

    def start(self):
        signal.signal(signal.SIGINT, self._signal_handler)
        self.is_scanning = True
        print("LiDAR scan started. Close the plot window and press Ctrl+C to stop.")
        try:
            for scan in self.lidar.iter_scans(max_buf_meas=ANGLE_RES):
                # populate distances by integer degree
                self.distances = [0] * ANGLE_RES
                for (_, angle, distance) in scan:
                    idx = min(ANGLE_RES-1, floor(angle))
                    self.distances[idx] = distance
                self.process_scan(self.distances)
        except Exception as e:
            print(f"ERROR during scan: {e}", file=sys.stderr)
        finally:
            self.stop()

    def process_scan(self, distances):
        """
        Convert polar distances→(x,y), update scatter plot.
        """
        xs, ys = [], []
        for deg, dist in enumerate(distances):
            if dist > 0:
                θ = radians(deg)
                xs.append(dist * cos(θ))
                ys.append(dist * sin(θ))
        # Update scatter data
        self.scatter.set_offsets(list(zip(xs, ys)))
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def stop(self):
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
