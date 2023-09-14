import pigpio
from time import sleep

ESC_PIN = 14

pi = pigpio.pi()

print("Calibrating...")
pi.set_servo_pulsewidth(ESC_PIN, 2400)
input("Connect power and press Enter to calibrate...")
pi.set_servo_pulsewidth(ESC_PIN, 2400)   # Official docs: "about 2 seconds".
sleep(2)
pi.set_servo_pulsewidth(ESC_PIN, 650)   # Time enough for the cell count, etc. beeps to play.
sleep(4)
print("Finished calibration.")