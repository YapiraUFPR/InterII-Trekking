import board
import busio
import adafruit_pca9685
from time import sleep

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

pca.frequency = 50
esc_channel = pca.channels[0]

import adafruit_motor.servo
servo = adafruit_motor.servo.Servo(esc_channel)

print("will start")

kit.servo[0].angle = 90
sleep(3)

for i in range(90, 180, 5):
    kit.servo[0].angle = i
    sleep(0.5)

for i in range(180, 90, -5):
    kit.servo[0].angle = i
    sleep(0.5)

sleep(1)
kit.servo[0].angle = 90