from machine import Pin
from time import sleep
from Utils import SeeFunctions, ThinkFunctions, ActFunctions
import math

# ------------------------- Encoder Setup ------------------------- #
tick_count_1 = 0
last_a_1 = 0
tick_count_2 = 0
last_a_2 = 0

# Encoder 1 ISR
def update_encoder1(pin):
    global tick_count_1, last_a_1
    a = pin_a1.value()
    b = pin_b1.value()
    if a != last_a_1:
        direction = 1 if a != b else -1
        tick_count_1 += direction
        last_a_1 = a

# Encoder 2 ISR
def update_encoder2(pin):
    global tick_count_2, last_a_2
    a = pin_a2.value()
    b = pin_b2.value()
    if a != last_a_2:
        direction = 1 if a != b else -1
        tick_count_2 += direction
        last_a_2 = a

# Encoder Pins
pin_a1 = Pin(18, Pin.IN)
pin_b1 = Pin(13, Pin.IN)
pin_a2 = Pin(12, Pin.IN)
pin_b2 = Pin(5, Pin.IN)

# Attach encoder interrupts
pin_a1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder1)
pin_a2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder2)

# ---------------------- Constants and Variables ---------------------- #
PPR = 16  # Pulses per revolution
GEAR_RATIO = 120
WHEEL_DIAMETER_CM = 6.5
WHEEL_BASE_CM = 15.5
recognition_distance = 200  # mm

x, y, theta = 0.0, 0.0, 0.0  # Initial position (cm, radians)

IR_sensor_pins = [36, 34, 35, 4, 39]
counter = 0
COUNTER_MAX = 5
COUNTER_STOP = 50

current_state = 'forward'
object_detected = False
state_updated = True
left_Speed = 0
right_Speed = 0

# --------------------------- Initialization --------------------------- #
SeeFunctions.setup_ir_sensors(*IR_sensor_pins)
SeeFunctions.setup_VL53L0X()
ActFunctions.motor_setup(26, 27)

# ----------------------------- Main Loop ----------------------------- #
while True:
    # ----------------------------- See ----------------------------- #
    distance_mm = SeeFunctions.TOFdistance() 
    if distance_mm < recognition_distance:
        object_detected = True

    sensor_vals = SeeFunctions.read_binary_values()
    print("IR Binary Values:", sensor_vals)

    # Encoder feedback
    delta_ticks_left = tick_count_1
    delta_ticks_right = tick_count_2
    tick_count_1 = 0
    tick_count_2 = 0

    left_distance_cm = (delta_ticks_left / PPR / GEAR_RATIO) * math.pi * WHEEL_DIAMETER_CM
    right_distance_cm = (delta_ticks_right / PPR / GEAR_RATIO) * math.pi * WHEEL_DIAMETER_CM

    # ---------------------------- Think ---------------------------- #
    delta_d = (right_distance_cm + left_distance_cm) / 2.0
    delta_theta = (right_distance_cm - left_distance_cm) / WHEEL_BASE_CM

    theta += delta_theta
    x += delta_d * math.cos(theta)
    y += delta_d * math.sin(theta)

    # error = ThinkFunctions.compute_error(sensor_vals, method='binary')

    # ----------------------------- Act ----------------------------- #
    print(f"Pose: x={x:.2f} cm, y={y:.2f} cm, θ={math.degrees(theta):.2f}°")
    print(f"distance: {distance_mm:.2f}")
    ActFunctions.motor_speed(left_Speed, right_Speed)
    print(counter)
    counter += 1
    sleep(0.1)

    # ------------------- State Machine Placeholder ------------------ #
    """
    # Implement the line-following state machine transitions
    if current_state == 'forward':
        counter = 0
        if line_right and not line_left:
            current_state = 'turn_right'
            state_updated = True
        elif line_left and not line_right:
            current_state = 'turn_left'
            state_updated = True
        elif line_left and line_right and line_center:  # lost the line
            current_state = 'turn_left'
            state_updated = True
        elif line_left and line_center and not line_right:
            current_state = 'forward'
            state_updated = True
        elif button_right.value() == True:
            current_state = 'stop'
            state_updated = True

    if current_state == 'turn_right':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
        elif button_right.value() == True:
            current_state = 'stop'
            state_updated = True

    if current_state == 'turn_left':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
        elif button_right.value() == True:
            current_state = 'stop'
            state_updated = True

    if current_state == 'stop':
        led_board.value(1)
        if counter >= COUNTER_STOP:
            current_state = 'forward'
            state_update = True
            led_board.value(0)
    """

