import Jetson.GPIO as GPIO
import time
from sys import argv

class TCS3200:
    NUM_CYCLES = 10

    def __init__(self, s2, s3, signal):
        self.s2 = s2
        self.s3 = s3 
        self.signal = signal 

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(signal, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(s2, GPIO.OUT)
        GPIO.setup(s3, GPIO.OUT)

    def read(self):

        rgb = []
        for signal_s2, signal_s3 in [(GPIO.LOW, GPIO.LOW), (GPIO.HIGH, GPIO.HIGH),  (GPIO.LOW, GPIO.HIGH)]:

            GPIO.output(self.s2, signal_s2)
            GPIO.output(self.s3, signal_s3)
            time.sleep(0.002)
            start = time.time()

            for _ in range(self.NUM_CYCLES):
                GPIO.wait_for_edge(self.signal, GPIO.FALLING)

            duration = time.time() - start 
            color  = self.NUM_CYCLES / duration  
            rgb.append(color)

        return rgb

    def __del__(self):
        GPIO.cleanup()

if __name__=='__main__':
    tcs32 = TCS3200(int(argv[1]), int(argv[2]), int(argv[3]))

    while 1:
        rgb = tcs32.read()
        print(rgb)
        time.sleep(5)