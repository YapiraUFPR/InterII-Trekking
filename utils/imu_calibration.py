import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_GAME_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from time import sleep

SAMPLE_RATE = 400
i2c = busio.I2C(board.SCL, board.SDA, frequency=SAMPLE_RATE*1000)
bno = BNO08X_I2C(i2c,address=0x4b) # BNO080 (0x4b) BNO085 (0x4a)

bno.initialize()
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

print(
    """
    Perform the accelerometer calibration motions. The accelerometer will be calibrated after the device is moved into 4-6 unique orientations and held in each orientation for ~1 second. One way to think about this is the “cube” method described below:
        a. Imagine that the device is a standard cube with 6 faces (Front, Back, Left, Right, Top, Bottom). See Figure 2 for visualization.
        b. Orient the device so that it is sitting on each face of the cube sequentially. For example, start with the device positioned normally (bottom face of the cube down). Then orient it so that the right side of the cube is facing down. Then continue through the rest of the faces of the cube. See Figure 3 for an example.
        c. Hold the device in each position for 1 second
        d. Notes about the accelerometer calibration motions:
            i. If one of the faces is difficult to position the device, you do not need to do all 6 faces (e.g. if cables or straps are in  the way). Four or five orientations should be sufficient.
            ii. The positions do not need to be perfectly aligned with the cube faces. The important part is that you position the device into 4-6 unique orientations. The cube is merely a way to visualize the type of motions required.
            iii. It does not matter which order the cube face positions are moved into
    4. Perform the gyroscope calibration
        a. Set the device down on a stationary surface for ~2-3 seconds to calibrate the gyroscope
    5. Perform the magnetometer calibration motions
        a. Rotate the device ~180° and back to the beginning position in each axis (roll, pitch, yaw) as shown in Figure 1
            i. The speed of the rotation should be approximately ~2 seconds per axis but it does not need to be perfect 
        b. Observe the Status bit of the magnetic field output
        c. Continue rotations until the Magnetic Field Status bit reads 2 or 3 (medium or high accuracy)
    6. When finished with the calibration motions, run the Save DCD Now command, which will save this calibration data into flash"""
)

bno.begin_calibration()
callib_status = bno.calibration_status
while callib_status != 3:
    print("Calibration status: ", callib_status)
    sleep(1/SAMPLE_RATE)
    callib_status = bno.calibration_status
    
bno.save_calibration_data()