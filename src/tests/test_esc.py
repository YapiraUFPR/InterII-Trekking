#!/usr/bin/python3
# SPDX-FileCopyrightText: 2018 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""CircuitPython Essentials Servo standard servo example"""
import time
import board
import pwmio
from adafruit_motor.servo import ContinuousServo

ESC_PIN = board.D4

# create a PWMOut object on Pin A2.
pwm = pwmio.PWMOut(ESC_PIN, frequency=800)

# Create a servo object, my_servo.
esc = ContinuousServo(pwm, min_pulse=500, max_pulse=2500)

while True:
    for t in range(0, 100, 1):  # 0 - 180 degrees, 5 degrees at a time.
        esc.throttle = t/100
        time.sleep(0.05)
    for angle in range(100, -100, 1): # 180 - 0 degrees, 5 degrees at a time.
        esc.throttle = t/100
        time.sleep(0.05)
