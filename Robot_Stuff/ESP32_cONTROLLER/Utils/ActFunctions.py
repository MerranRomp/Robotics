from machine import Pin, PWM
from time import sleep


global pwm1, pwm2
def motor_setup(motorA, motorB):
    pwm1 = PWM(Pin(motorA), freq=1000)
    pwm2 = PWM(Pin(motorB), freq=1000)

def motor_speed(left_Speed, right_Speed):
    pwm1.duty(int(left_Speed * 1023))  # 0.0–1.0
    pwm2.duty(int(right_Speed * 1023))