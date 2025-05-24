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

## Cloning Repository and Requirements

git clone https://github.com/FonzyShock/CIWS.git
cd CIWS

Requirements:
- Python 3.x
- OpenCV (for image processing tasks)
- NumPy
- Lidar sensor and associated Python libraries
- Camera module compatible with OpenCV
- Actuators (e.g., servos) and corresponding control libraries
- RPi.GPIO or appropriate GPIO libraries for your platform


## Program Execution

In the command line
python3 CIWSControl.py

Ensure all hardware components (sensors, actuators, camera) are properly connected and configured before running the script.

## Contributing:
Contributions are welcome! Please fork the repository and submit a pull request with your enhancements or bug fixes.

Contact:
For any questions or suggestions, please contact the repository owner via GitHub.





