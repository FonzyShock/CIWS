# !/usr/bin/python3
# Servo_test.py created by alfon at 5/29/2025

# Enter feature description here

# Python Libraries
from gpiozero import Zero
from time import sleep

servo = Servo(17)
def main():
    while True:
        servo.min()
        sleep(2)
        servo.mid()
        sleep(2)
        servo.max
        sleep(2)

# Python Functions
# Define Functions Here
if __name__ == '__main__':
    pass
