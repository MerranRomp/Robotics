from machine import Pin, PWM

# Declare globals at module level
pwm1 = None
pwm2 = None

def motor_setup(motorA, motorB):
    global pwm1, pwm2

    pwm1 = PWM(Pin(motorA), freq=1000)
    pwm2 = PWM(Pin(motorB), freq=1000)

def motor_speed(left_speed, right_speed):
    global pwm1_fwd, pwm1_rev, pwm2_fwd, pwm2_rev

    def set_pwm(forward_pwm, reverse_pwm, value):
        value = max(min(value, 1.0), -1.0)  # clamp to [-1, 1]
        if value >= 0:
            forward_pwm.duty(int(value * 1023))
            reverse_pwm.duty(0)
        else:
            forward_pwm.duty(0)
            reverse_pwm.duty(int(-value * 1023))

    print("L:", left_speed, "R:", right_speed)
    set_pwm(pwm1_fwd, pwm1_rev, left_speed)
    set_pwm(pwm2_fwd, pwm2_rev, right_speed)

