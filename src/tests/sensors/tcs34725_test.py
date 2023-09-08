import board
import busio
from adafruit_tcs34725 import TCS34725
from time import sleep

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
tcs = TCS34725(i2c,address=0x29) #0x29

while True:
    r, g, b = tcs.color_rgb_bytes
    temp = tcs.color_temperature
    lux = tcs.lux

    print(f"Color: ({r}, {g}, {b})")
    print(f"Temperature: {temp}K")
    print(f"Lux: {lux}")
    print("")

    sleep(1)