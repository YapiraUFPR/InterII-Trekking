import board
import busio
import adafruit_vl53l0x
from time import sleep 

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
sensor = adafruit_vl53l0x.VL53L0X(i2c, address=0x29)

while True:

    print(f'Range: {sensor.range}mm')
    sleep(1)