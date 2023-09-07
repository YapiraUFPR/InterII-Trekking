# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
import adafruit_bitbangio as bitbangio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
# from IMU_tracker import IMUTracker
import numpy as np

SAMPLE_RATE = 400

i2c = busio.I2C(scl=board.SCL, sda=board.SDA, frequency=SAMPLE_RATE*1000)
bno = BNO08X_I2C(i2c,address=0x4b) # BNO080 (0x4b) BNO085 (0x4a)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

# tracker = IMUTracker(sampling=SAMPLE_RATE)
# callib_data = []
# finished_callib = False
# while not finished_callib:
#     time.sleep(1/SAMPLE_RATE)
#     accel_x, accel_y, accel_z = bno.acceleration  
#     gyro_x, gyro_y, gyro_z = bno.gyro 
#     mag_x, mag_y, mag_z = bno.magnetic 
#     # quat_i, quat_j, quat_k, quat_real = bno.quaternion  
#     callib_data.append([gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z])
#     if len(callib_data) > 1000:
#         finished_callib = True

# callib_data = np.array(callib_data, dtype=np.float32)
# tracker.initialize(callib_data)

print("position tracking started.")
init_time = time.time()
out_file = np.array([], dtype=np.float32)

# try:
while True:

    time.sleep(1/SAMPLE_RATE)
    accel_x, accel_y, accel_z = bno.acceleration  
    # print("Acceleration: X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
    gyro_x, gyro_y, gyro_z = bno.gyro 
    # print("Gyro: X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
    mag_x, mag_y, mag_z = bno.magnetic 
    # print("Magnetometer: X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  
    # print("RVQ: I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real))

    # data.append([gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, quat_i, quat_j, quat_k, quat_real])
    data = [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, quat_i, quat_j, quat_k, quat_real]

    # np.append(out_file, np.array(data, dtype=np.float32))
    # p = tracker.attitudeTrack(data)[0]
    print(time.time(), data)

    # if time.time() - init_time > 30:
    #     break

# finally:
#     print(out_file)
#     np.save("data.npy", out_file)
# data = np.array(data, dtype=np.float32)
# np.save("data.npy", data)