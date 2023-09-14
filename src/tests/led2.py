import board
import digitalio
from sys import argv

led = digitalio.DigitalInOut(board.D4)
led.switch_to_output(value=False)

if argv[1] == "on":
    led.value = True
else:
    led.value = False
