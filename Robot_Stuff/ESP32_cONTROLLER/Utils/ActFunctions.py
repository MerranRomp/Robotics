from machine import Pin, PWM

# Declare globals at module level
pwm1 = None
pwm2 = None

def motor_setup(motorA, motorB):
    global pwm1, pwm2
    pwm1 = PWM(Pin(motorA), freq=1000)
    pwm2 = PWM(Pin(motorB), freq=1000)

def motor_speed(left_Speed, right_Speed):
    global pwm1, pwm2
    if pwm1 and pwm2:
        pwm1.duty(int(left_Speed * 1023))  # convert 0.0–1.0 to 0–1023
        pwm2.duty(int(right_Speed * 1023))
