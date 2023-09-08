import board
import busio
import adafruit_bitbangio as bitbangio
from digitalio import DigitalInOut

from adafruit_vl53l0x import VL53L0X
from adafruit_tcs34725 import TCS34725
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)

import numpy as np
from time import sleep

SAMPLE_RATE = 400

bno_i2c = busio.I2C(scl=board.SCL, sda=board.SDA, frequency=SAMPLE_RATE*1000)
bno = BNO08X_I2C(bno_i2c,address=0x4b) # BNO080 (0x4b) BNO085 (0x4a)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

tcs_i2c = bitbangio.I2C(scl=board.D27, sda=board.D22, frequency=SAMPLE_RATE*1000)
tcs = TCS34725(tcs_i2c,address=0x29) #0x29
tcs.gain = 60
tcs.integration_time = 200

vl5_i2c = bitbangio.I2C(scl=board.D5, sda=board.D6, frequency=SAMPLE_RATE*1000)
vl5 = VL53L0X(vl5_i2c, address=0x29)

LED_PIN = board.D4
led = DigitalInOut(LED_PIN)
led.switch_to_output(value=False)

while True:
    r, g, b = tcs.color_rgb_bytes
    temp = tcs.color_temperature
    lux = tcs.lux

    print(f"Color: ({r}, {g}, {b})")
    print(f"Temperature: {temp}K")
    print(f"Lux: {lux}")
    print("")

    dist = vl5.range
    print(f"Distance: {dist}")
    print("")

    accel_x, accel_y, accel_z = bno.acceleration  
    print("Acceleration: X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
    gyro_x, gyro_y, gyro_z = bno.gyro 
    print("Gyro: X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
    mag_x, mag_y, mag_z = bno.magnetic 
    print("Magnetometer: X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  
    print("RVQ: I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real))
    print("")

    print("Will blink led")
    led.value = True
    sleep(0.5)
    led.value = False
    sleep(0.5)
    led.value = True
    sleep(0.5)
    led.value = False

    sleep(1)