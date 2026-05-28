# !/usr/bin/python3
# Servo_test.py created by alfon at 5/29/2025

# Enter feature description here

# Python Libraries
import  RPi.GPIO as GPIO
import time

servo_pin = 23
freq = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

pwm = GPIO.PWM(servo_pin, freq)

pwm.start(0)

def set_angle(angle):
	duty = 2 + (angle / 18)
	pwm.ChangeDutyCycle(duty)

try:
	while True:
		for angle in range(0,181,1):
			set_angle(angle)
			time.sleep(0.01)
			print(angle)

		for angle in range (181,0,-1):
			set_angle(angle)
			time.sleep(0.01)
			print(angle)
		pass

except KeyboardInterrupt:
	print("Interrupted")
finally:
	set_angle(0)
	time.sleep(0.1)
	print("Done")
#	GPIO.cleanup()
