#!/usr/bin/env python3
# kill_motors.py
# Emergency motor shutdown for differential‐drive L298N (no PWM control)
# Author: Alfonso S.
# Date:   2025-06-10

import time
import board
from digitalio import DigitalInOut, Direction
from pwmio import PWMOut

def kill_motors():
    """
    Immediately disable all motor driver inputs:
      • IN1 = D26, IN2 = D13 (Motor A)
      • IN3 = D21, IN4 = D27 (Motor B)
      • PWM A = D19, PWM B = D11
    """
    # 1) Configure direction pins and drive them low
    dir_pins = []
    for pin_id in (board.D26, board.D13, board.D21, board.D27):
        pin = DigitalInOut(pin_id)
        pin.direction = Direction.OUTPUT
        pin.value = False
        dir_pins.append(pin)

    # 2) Configure PWM outputs and ensure zero duty cycle
    pwmA = PWMOut(board.D19, frequency=1000, duty_cycle=0)
    pwmB = PWMOut(board.D11, frequency=1000, duty_cycle=0)

    # Short delay to allow hardware to settle
    time.sleep(0.05)

    # 3) Deinitialize all pins to release resources
    for pin in dir_pins:
        pin.deinit()
    pwmA.deinit()
    pwmB.deinit()

    print(">>> KILL CODE: motors disabled and all outputs set LOW <<<")

if __name__ == "__main__":
    kill_motors()
