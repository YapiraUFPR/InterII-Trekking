import Jetson.GPIO as GPIO
from time import sleep

LED_PIN1 = 11
LED_PIN2 = 13
LED_PIN3 = 32

#GPIO.cleanup()

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_PIN1, GPIO.OUT, initial=GPIO.LOW) # verde
GPIO.setup(LED_PIN2, GPIO.OUT, initial=GPIO.LOW) # azul
GPIO.setup(LED_PIN3, GPIO.OUT, initial=GPIO.LOW) # vermelho

#sleep(2)

#GPIO.output(LED_PIN1, GPIO.HIGH)
#GPIO.output(LED_PIN2, GPIO.HIGH)
#GPIO.output(LED_PIN3, GPIO.HIGH)

sleep(5)

p = GPIO.PWM(32, 50)
val = 0
incr = 5
p.start(val)

sleep(1)

print("PWM running. Press CTRL+C to exit.")
try:
    while True:
        sleep(0.25)
        if val >= 100:
            incr = -incr
        if val <= 0:
            incr = -incr
        val += incr
        p.ChangeDutyCycle(val)
finally:
    p.stop()
    GPIO.cleanup()

sleep(5)

p.stop()
GPIO.cleanup()