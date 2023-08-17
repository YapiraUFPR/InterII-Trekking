#!/usr/bin/env python3

# Node to control the LED strip using the TCS34725 color sensor
# LED strip should light up when color sensor detects the mark color
# Author: Gabriel Pontarolo

from adafruit_tcs34725 import TCS34725
import board
import busio
from digitalio import DigitalInOut 
from sys import argv
from time import sleep

MARK_COLOR_LOWER = [255, 255, 255]
MARK_COLOR_UPPER = [200, 200, 200]
TCS_ADDR = 0x29
LED_PIN = board.D13

def main():

    # sensor initialization
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    tcs = TCS34725(i2c,address=TCS_ADDR) #0x29

    # led initialization
    led_pin = DigitalInOut(LED_PIN)

    led_on = False
    while True:

        r, g, b = tcs.color_rgb_bytes
        temp = tcs.color_temperature
        lux = tcs.lux

        color = [r, g, b]

        if not led_on and all(MARK_COLOR_LOWER[i] < color[i] < MARK_COLOR_UPPER[i] for i in range(3)):
            led_pin.value = True
            led_on = True

        print(f"Color: {color}")
        print(f"Temperature: {temp}")
        print(f"Lux: {lux}")
        print()
        
        sleep(0.5)


if __name__ == "__main__":
    main()