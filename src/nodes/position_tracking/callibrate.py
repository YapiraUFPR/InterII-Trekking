import time
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
# from IMU_tracker import IMUTracker
import numpy as np
from digitalio import DigitalInOut

SAMPLE_RATE = 400

i2c = busio.I2C(board.SCL, board.SDA, frequency=SAMPLE_RATE*1000)
bno = BNO08X_I2C(i2c,address=0x4b) # BNO080 (0x4b) BNO085 (0x4a)

# rst = DigitalInOut(board.D4)
# rst.switch_to_output(value=False)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

bno.begin_calibration()
status = bno.calibration_status
while status != 3:
    status = bno.calibration_status
    print(f"Callib status {status}")
    time.sleep(1/SAMPLE_RATE)

bno.save_calibration()