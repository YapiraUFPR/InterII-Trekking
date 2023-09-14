import pigpio 
from time import sleep

class Esc:

    def __init__(self, pin, min_pulse=500, max_pulse=2500):
        
        try:
            self.pi = pigpio.pi()
        except Exception:
            print("Couldn't initialize esc. Did you run 'sudo pigpio'?")
            exit(1)

        self.pin = pin
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.half_pulse = (max_pulse - min_pulse) // 2

        self.pi.set_servo_pulsewidth(self.pin, 0)
        sleep(5)
        self.pi.set_servo_pulsewidth(self.pin, self.half_pulse)
        self.speed = 0
        print("Esc initialized")

    # @property
    # def speed(self):
    #     """
    #     Motor speed. Between -100 to 100.
    #     """
    #     return self.speed

    # @speed.setter
    def set_speed(self, value):
        self.speed = 100 if value > 100 else value
        self.speed = -100 if value < -100 else value

        pwm = self.speed*10 + 1500
        self.pi.set_servo_pulsewidth(self.pin, pwm)
        
    def stop(self):
        self.pi.set_servo_pulsewidth(self.pin, self.half_pulse)

    def __del__(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)
        # self.pi.stop()
        print("Esc stopped.")