# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
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

SAMPLE_RATE = 400

i2c = busio.I2C(board.SCL, board.SDA, frequency=SAMPLE_RATE*1000)
bno = BNO08X_I2C(i2c,address=0x4b) # BNO080 (0x4b) BNO085 (0x4a)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

bno.begin_calibration()
while bno.calibration_status != 3:
    print("Calibration status: ", bno.calibration_status)
    time.sleep(1/SAMPLE_RATE)
    
bno.save_calibration_data()