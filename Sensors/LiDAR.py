#!/usr/bin/env python3
"""
simple_test_rplidar.py

A very basic script that:
  1. Opens the RPLidar on a user-specified serial port.
  2. Prints the first 10 (angle, distance) measurements it sees.
  3. Cleans up and exits.

Edit PORT_NAME below to match your system (e.g. '/dev/ttyUSB0', '/dev/tty.SLAB_USBtoUART', or 'COM3').
"""

from rplidar import RPLidar

# ─── CONFIG ─────────────────────────────────────────────────────────────────────

# Change this line to your LiDAR’s port:
PORT_NAME = 'COM7' # '/dev/ttyUSB0'    # e.g. '/dev/ttyUSB0' on Linux, 'COM3' on Windows, etc.

# How many readings to print before exiting
NUM_TO_PRINT = 10

# ─── MAIN ────────────────────────────────────────────────────────────────────────

def main():
    try:
        print(f"[*] Opening RPLidar on port {PORT_NAME} …")
        lidar = RPLidar(PORT_NAME)
    except Exception as e:
        print(f"[!] Could not open LiDAR on {PORT_NAME}.")
        print("    • Make sure the cable is plugged in, the port name is correct, and no other program is using it.")
        print(f"    • Error detail: {e}")
        return

    try:
        print("[*] LiDAR opened successfully.")
        print("[*] Reading first", NUM_TO_PRINT, "measurements…\n")

        count = 0
        # iter_measurables() yields (quality, angle, distance_mm)
        for quality, angle, dist_mm in lidar.iter_measurables():
            angle = round(angle, 2)
            dist_m = round(dist_mm / 1000.0, 3)
            print(f"  → #{count+1:2d}: angle = {angle:6.2f}°, distance = {dist_m:5.3f} m, quality = {quality}")
            count += 1
            if count >= NUM_TO_PRINT:
                break

    except KeyboardInterrupt:
        print("\n[!] Interrupted by user.")
    except Exception as e:
        print(f"\n[!] LiDAR error during scanning: {e}")
    finally:
        print("\n[*] Stopping LiDAR and closing port …")
        try:
            lidar.stop()
            lidar.disconnect()
        except:
            pass
        print("[*] Done.")

if __name__ == '__main__':
    main()
