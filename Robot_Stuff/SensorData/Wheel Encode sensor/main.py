# code to test the motor wheel encoder
# running on a esp32 with micropython
#reporting the speed of the wheel in cm/s with the wheel diameter in cm and a gearbox ratio accounted for
# will need calibration for the number of pulses per revolution (PPR) of the encoder
# and the gearbox ratio and the wheel diameter.


from machine import Pin
from time import ticks_us, ticks_diff, sleep
import micropython
import math

# === CONFIGURATION ===
PPR = 11  # encoder pulses per motor revolution (check your encoder datasheet)
GEAR_RATIO = 30  # gearbox reduction ratio
WHEEL_DIAMETER_CM = 6.5

# === Encoder Pins ===
pin_a = Pin(18, Pin.IN)
pin_b = Pin(19, Pin.IN)

# === Runtime Variables ===
tick_count = 0
last_a = 0

# === ISR: Count pulses ===
def update(pin):
    global tick_count, last_a
    a = pin_a.value()
    b = pin_b.value()
    if a != last_a:  # only on change
        direction = 1 if a != b else -1
        tick_count += direction
        last_a = a

# Attach interrupt
pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update)

# === Main Loop ===
while True:
    start_ticks = tick_count
    start_time = ticks_us()
    sleep(0.5)  # measure over 0.5s
    delta_ticks = tick_count - start_ticks
    delta_time = ticks_diff(ticks_us(), start_time) / 1_000_000  # seconds

    # Compute wheel speed
    motor_rps = delta_ticks / PPR / delta_time
    wheel_rps = motor_rps / GEAR_RATIO
    wheel_circ = math.pi * WHEEL_DIAMETER_CM
    speed_cm_s = wheel_rps * wheel_circ

    print("Speed: {:.2f} cm/s | Wheel RPS: {:.2f}".format(speed_cm_s, wheel_rps))

