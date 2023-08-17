#!/usr/bin/python3
import time
import board
import pwmio

ESC_PIN = board.D2
PWM_MAX = 65535

pwm = pwmio.PWMOut(ESC_PIN, frequency=50)

while True:
    for duty_cycle in range(0, 100, 5): 
        pwm.duty_cycle = int(duty_cycle / 100 * PWM_MAX)
        print(f'Duty Cycle: {duty_cycle}')
        time.sleep(1)