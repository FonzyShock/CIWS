# CIWS: Close-In Weapon System Nerf Gun Turret

This repository contains the full source code and supporting files for the CIWS (Close-In Weapon System) project for the Maker's Club, a programmable Nerf gun turret capable of autonomous navigation, target detection, tracking, and engagement. Inspired by naval defense systems, this educational platform demonstrates real-time control systems, sensor fusion, and actuation.

---

## Overview

The CIWS system is designed to emulate a naval-style autonomous turret for rapid target acquisition and engagement. It employs computer vision, motion control algorithms, and actuator commands to simulate real-world engagement scenarios in a safe and controlled environment using a Nerf gun.

### Key Features
- Lidar-based autonomous navigation
- Real-time object tracking using camera-based input
- Target prioritization and engagement routines
- Modular architecture for easy adaptation and extension
- Fully autonomous control with manual override capability 

---

## Repository Structure

```plaintext
CIWS/
├── Actuators/           # Scripts and modules for motor and firing control
├── Sensors/             # Scripts for sensor data acquisition (e.g., camera input)
├── CIWSControl.py       # Main system integrator and execution script
└── README.md            # Project documentation

