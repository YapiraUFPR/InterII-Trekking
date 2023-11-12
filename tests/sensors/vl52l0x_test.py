import board
import busio
import adafruit_vl53l0x
from time import sleep 

import adafruit_bitbangio as bitbangio

i2c = bitbangio.I2C(scl=board.D27, sda=board.D17, frequency=400000)


# i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
sensor = adafruit_vl53l0x.VL53L0X(i2c, address=0x29)

while True:

    print(f'Range: {sensor.range}mm')
    sleep(1)