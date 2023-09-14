
#!/usr/bin/env python3

# Node to control the LED strip using the TCS34725 color sensor
# LED strip should light up when color sensor detects the mark color
# Author: Gabriel Pontarolo

import board
import pwmio
from adafruit_motor import servo
import adafruit_bitbangio as bitbangio
from scripts.libs.esc import Esc

from adafruit_tcs34725 import TCS34725
from digitalio import DigitalInOut 
from sys import argv
from time import sleep

MARK_COLOR_LOWER = [20, 20, 0]
MARK_COLOR_UPPER = [30, 30, 5]
TCS_ADDR = 0x29
LED_PIN = board.D4

SAMPLE_RATE = 400

ESC_PIN = 14
SERVO_PIN = board.D18 

LINEAR_SPEED = 10
STALL_SPEED = 20

def main():
    esc =  Esc(ESC_PIN, 650, 2400)
    servo_pwm = pwmio.PWMOut(SERVO_PIN, frequency=50)
    servo_motor = servo.Servo(servo_pwm)
    servo_motor.angle = 90
    angle =  90

    # sensor initialization
    tcs_i2c = bitbangio.I2C(scl=board.D27, sda=board.D22, frequency=SAMPLE_RATE*1000)
    tcs = TCS34725(tcs_i2c,address=0x29) #0x29
    tcs.gain = 60
    tcs.integration_time = 200

    # led initialization
    led_pin = DigitalInOut(LED_PIN)
    led_pin.switch_to_output(value=False)
    led_on = False

    print("Accelerating ...")
    for i in range(0, STALL_SPEED):
        esc.set_speed(i)
        sleep(0.01)

    print("Continuous speed...")
    for i in range(STALL_SPEED, LINEAR_SPEED, -1):
        esc.set_speed(i)
        sleep(0.01)
    
    sleep(3)
    servo_motor.angle = 130
    sleep(0.5)
    servo_motor.angle = 90
    cur_speed = LINEAR_SPEED
    found_base = False
    print("Color loop")
    while True:
        r, g, b = tcs.color_rgb_bytes
        temp = tcs.color_temperature
        lux = tcs.lux

        color = [r, g, b]

        if not led_on and all(MARK_COLOR_LOWER[i] < color[i] < MARK_COLOR_UPPER[i] for i in range(3)):
            led_pin.value = True
            led_on = True
            print("Color found")
            if not found_base:
                print("Will begin deaccelerating")
                found_base = True

        if found_base:
            print("Deaccelerating")
            cur_speed -= 1
            esc.set_speed(cur_speed)

        if cur_speed == 0:
            led.value = False
            break

        sleep(0.01)

try:
    main()
except Exception:
    esc.set_speed(0)
finally:
    led.value = False
