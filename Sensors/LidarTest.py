#!/usr/bin/env python3
"""
Basic RPLidar test for Raspberry Pi 5

1. Auto-detects your RPLidar’s serial port.
2. Initializes the LiDAR over USB only.
3. Prints a single full 360° scan of (angle, distance, quality).
"""

import serial.tools.list_ports
from rplidar import RPLidar, RPLidarException

def find_lidar_port():
    """Return the first USB/CP210x serial port found, or None."""
    for port in serial.tools.list_ports.comports():
        desc = port.description.lower()
        if any(x in desc for x in ('usb', 'cp210', 'silicon')):
            return port.device
    return None

def main():
    port = find_lidar_port()
    if port is None:
        print("No LiDAR port found. Is it plugged in via USB?")
        return
    print(f"Found LiDAR on {port}")

    lidar = RPLidar(port)

    try:
        # Device info
        info = lidar.get_info()
        print("\n=== LIDAR INFO ===")
        for k, v in info.items():
            print(f"{k}: {v}")

        # Health status
        health = lidar.get_health()
        status, error = health[0], health[1]
        print("\n=== HEALTH ===")
        print(f"status={status}, error_code={error}")
        if status != 0:
            print("Health NOT OK — check power or connections.")

        # One full 360° scan
        print("\nScanning one full revolution (Ctrl+C to stop)...")
        for scan in lidar.iter_scans():
            print("\n--- New 360° Scan ---")
            for quality, angle, dist in scan:
                print(f"{angle:6.2f}° | {dist:5.0f} mm | Q={quality}")
            break  # remove this to keep scanning

    except KeyboardInterrupt:
        print("Stopped by user.")
    except RPLidarException as e:
        print(f"LiDAR error: {e}")
        print("Try a powered USB hub or a different USB port.")
    finally:
        print("\nDisconnecting LiDAR...")
        try:
            lidar.stop()
            lidar.disconnect()
        except:
            pass
        print("Done.")

if __name__ == "__main__":
    main()
