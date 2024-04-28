import board
import digitalio
from sys import argv
from time import sleep

led = digitalio.DigitalInOut(board.D18)
led.switch_to_output(value=False)
led.value = True 
sleep(5)
led.value = False