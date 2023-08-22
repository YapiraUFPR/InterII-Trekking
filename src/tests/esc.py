#!/usr/bin/python3
import time
import board
import pwmio

ESC_PIN = board.D4
PWM_MAX = 65535

pwm = pwmio.PWMOut(ESC_PIN, frequency=50)

def dt2pwm(dt):
    return int(PWM_MAX * (dt/100))

def pw2dt(pw):
    return int(pw/(1/50))

while True:
    for pw in range(700, 2000): 
        # pwm.duty_cycle = int(duty_cycle / 100 * PWM_MAX)
        dc = pw2dt(pw)
        pwm.duty_cycle = dc 
        print(f'Duty Cycle: {dc}')
        time.sleep(1)


# pwm.duty_cycle = dt2pwm(75)
# time.sleep(1)
# pwm.duty_cycle = dt2pwm(60)
# time.sleep(1)
# pwm.duty_cycle = dt2pwm(30)
# time.sleep(1)