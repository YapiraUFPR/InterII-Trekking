from gpiozero import Servo 
from gpiozero.tools import sin_values
from time import sleep
from signal import pause

SIGNAL_PIN = 0

servo = Servo(SIGNAL_PIN)

servo.min()
sleep(1)
servo.max()
sleep(1)
servo.mid()
sleep(1)
servo.value = 0.5
sleep(0.5)
servo.value = 0.2
sleep(0.5)
servo.min()

servo.source = sin_values()
servo.source_delay = 0.1

pause()
