import os     #importing os library so as to communicate with the system
import time   #importing time library to make Rpi wait because its too impatient 
import pigpio #importing GPIO library
import board
import pwmio
from adafruit_motor import servo

ESC=4  #Connect the ESC in this GPIO pin 
SERVO_PIN = board.D17

pi = pigpio.pi();
pi.set_servo_pulsewidth(ESC, 0) 

max_value = 2000 #change this if your ESC's max value is different or leave it be
min_value = 700  #change this if your ESC's min value is different or leave it be

# for i in range(min_value, max_value):
#     pi.set_servo_pulsewidth(ESC, i)
#     print(i)
#     time.sleep(0.01)

pi.set_servo_pulsewidth(ESC, 1500)


# create a PWMOut object on Pin A2.
pwm = pwmio.PWMOut(SERVO_PIN, frequency=50)

# Create a servo object, my_servo.
servo_motor = servo.Servo(pwm, min_pulse=750, max_pulse=2250)

for k in range(3):
    for angle in range(0, 180, 5):  # 0 - 180 degrees, 5 degrees at a time.
        pi.set_servo_pulsewidth(ESC, 1400 + angle*2)
        #servo_motor.angle = angle
        time.sleep(0.05)
    for angle in range(0, 180, 5): # 180 - 0 degrees, 5 degrees at a time.
        pi.set_servo_pulsewidth(ESC, 1400 - angle*2)
        #servo_motor.angle = angle
        time.sleep(0.05)

pi.set_servo_pulsewidth(ESC, 0)
