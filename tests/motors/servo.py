#!/usr/bin/python3
# SPDX-FileCopyrightText: 2018 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""CircuitPython Essentials Servo standard servo example"""
import time
import board
import pwmio
from adafruit_motor import servo

SERVO_PIN = board.D4

# create a PWMOut object on Pin A2.
pwm = pwmio.PWMOut(SERVO_PIN, frequency=50)

# Create a servo object, my_servo.
servo_motor = servo.Servo(pwm, min_pulse=750, max_pulse=2250)

while True:
    for angle in range(0, 180, 5):  # 0 - 180 degrees, 5 degrees at a time.
        servo_motor.angle = angle
        time.sleep(0.05)
    for angle in range(180, 0, -5): # 180 - 0 degrees, 5 degrees at a time.
        servo_motor.angle = angle
        time.sleep(0.05)