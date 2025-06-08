# Motor Driver Control Example for esp32 with Micropython
# can give a direction and speed to the motor driver


from machine import Pin, PWM
from time import sleep

# PWM setup
pwm1 = PWM(Pin(25), freq=1000)
pwm2 = PWM(Pin(26), freq=1000)

def motor_forward(speed):
    pwm1.duty(int(speed * 1023))  # 0.0–1.0
    pwm2.duty(0)

def motor_backward(speed):
    pwm1.duty(0)
    pwm2.duty(int(speed * 1023))

def motor_stop():
    pwm1.duty(0)
    pwm2.duty(0)

# Test with speed control
motor_forward(0.5)
sleep(1)
motor_forward(1)
sleep(1)

motor_backward(0.5)
sleep(2)

motor_stop()

