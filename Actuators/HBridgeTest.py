#!/usr/bin/env python3
# hbridge_direction_test.py
# Simple forward/backward/off test with L298N (ENA/ENB jumpered to 5 V)

import time
import board
from digitalio import DigitalInOut, Direction

# —— Motor A direction pins ——  
IN1 = DigitalInOut(board.D27)  # A forward
IN2 = DigitalInOut(board.D21)  # A reverse

# —— Motor B direction pins ——  
IN3 = DigitalInOut(board.D13)  # B forward
IN4 = DigitalInOut(board.D26)   # B reverse

# Configure all as outputs, initialize LOW (off)
for pin in (IN1, IN2, IN3, IN4):
    pin.direction = Direction.OUTPUT
    pin.value = False

print("Starting H-bridge direction test…")

try:
    # 1) FORWARD
    print("→ FORWARD")
    IN1.value = True
    IN2.value = False
    IN3.value = True
    IN4.value = False
    time.sleep(2)

    # 2) BACKWARD
    print("→ BACKWARD")
    IN1.value = False
    IN2.value = True
    IN3.value = False
    IN4.value = True
    time.sleep(2)

    # 3) COAST/STOP
    print("→ COAST (all OFF)")
    IN1.value = False
    IN2.value = False
    IN3.value = False
    IN4.value = False
    time.sleep(2)

    print("Test complete.")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    # Ensure all lines are low before exit
    for pin in (IN1, IN2, IN3, IN4):
        pin.value = False
    print("Cleanup done.")
