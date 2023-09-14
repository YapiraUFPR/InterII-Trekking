import board
import digitalio
from sys import argv

led = digitalio.DigitalInOut(board.D4)
led.switch_to_output(value=False)
led.value = False
