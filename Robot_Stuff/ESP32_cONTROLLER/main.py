from machine import Pin, I2C, PWM
from time import sleep, ticks_ms, ticks_diff
from Utils import SeeFunctions, ThinkFunctions, ActFunctions
import math
import nodes

sleep(1)
# ------------------------- Encoder Setup ------------------------- #
tick_count_1 = 0
last_a_1 = 0
tick_count_2 = 0
last_a_2 = 0
i2c = I2C(0, scl=Pin(22), sda=Pin(21))  # adjust pins as needed


# Encoder 1 ISR
print("Init encoder 1")
def update_encoder1(pin):
    global tick_count_1, last_a_1
    a = pin_a1.value()
    b = pin_b1.value()
    if a != last_a_1:
        direction = 1 if a != b else -1
        tick_count_1 += direction
        last_a_1 = a

# Encoder 2 ISR
print("Init encoder 2")
def update_encoder2(pin):
    global tick_count_2, last_a_2
    a = pin_a2.value()
    b = pin_b2.value()
    if a != last_a_2:
        direction = 1 if a != b else -1
        tick_count_2 += direction
        last_a_2 = a

class Motor:
    def __init__(self, pin_fwd, pin_rev, freq=1000):
        self.pwm_fwd = PWM(Pin(pin_fwd))
        self.pwm_fwd.freq(freq)
        self.pwm_rev = PWM(Pin(pin_rev))
        self.pwm_rev.freq(freq)

    def set_speed(self, speed_percent):
        # Clamp to -100..100
        speed_percent = max(min(speed_percent, 100), -100)

        # Convert to 16-bit duty cycle (0..65535)
        duty = int(abs(speed_percent) / 100 * 65535)

        if speed_percent >= 0:
            self.pwm_fwd.duty_u16(duty)
            self.pwm_rev.duty_u16(0)
        else:
            self.pwm_fwd.duty_u16(0)
            self.pwm_rev.duty_u16(duty)

    def stop(self):
        self.pwm_fwd.duty_u16(0)
        self.pwm_rev.duty_u16(0)

motorA = Motor(pin_fwd=27, pin_rev=14)
motorB = Motor(pin_fwd=25, pin_rev=26)


# Encoder Pins
pin_a1 = Pin(18, Pin.IN)
pin_b1 = Pin(13, Pin.IN)
pin_a2 = Pin(12, Pin.IN)
pin_b2 = Pin(5, Pin.IN)

# Attach encoder interruptspin_a1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder1)
pin_a2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_encoder2)

# ---------------------- Constants and Variables ---------------------- #
PPR = 16  # Pulses per revolution
GEAR_RATIO = 120
WHEEL_DIAMETER_CM = 6.5
WHEEL_BASE_CM = 15.5
recognition_distance = 200  # mm

x, y, theta = 0.0, 0.0, 0.0  # Initial position (cm, radians)

IR_sensor_pins = [36, 34, 35, 32, 39]
counter = 0
COUNTER_MAX = 5
COUNTER_STOP = 50

object_detected = False
left_Speed = 0
right_Speed = 0
base_speed = 50  # Base speed in %
returning_to_node = False
state_entry_time = ticks_ms()

#statemachine variables
pickup_nodes = ['A1', 'A2', 'A3', 'A4']
dropoff_nodes = ['G6', 'G7', 'G8', 'G9']
start = 'F1'
goal = 'D5'
state = 'IDLE'
path = []
current_task = 0
last_node = None

path, cost = ThinkFunctions.dijkstra(start, goal)
print("Path:", path)
print("Total cost:", cost)



#robot starts at the first node
current_node = 'F1'

# --------------------------- Initialization --------------------------- #
print("Init sensors")
SeeFunctions.setup_ir_sensors(*IR_sensor_pins)
SeeFunctions.setup_VL53L0X(i2c)

# ----------------------------- Main Loop ----------------------------- #
while True:
    # ----------------------------- See ----------------------------- #
    distance_mm = SeeFunctions.TOFdistance() 
    sensor_vals = SeeFunctions.read_binary_values()
    # Encoder feedback
    delta_ticks_left = tick_count_1
    delta_ticks_right = tick_count_2
    tick_count_1 = 0
    tick_count_2 = 0
    # calculate speed based on encoder ticks
    left_distance_cm = (delta_ticks_left / PPR / GEAR_RATIO) * math.pi * WHEEL_DIAMETER_CM
    right_distance_cm = (delta_ticks_right / PPR / GEAR_RATIO) * math.pi * WHEEL_DIAMETER_CM
    # calculate speed in cm/s
    delta_d = (right_distance_cm + left_distance_cm) / 2.0
    delta_theta = (right_distance_cm - left_distance_cm) / WHEEL_BASE_CM
    # displacement in cm
    theta += delta_theta
    x += delta_d * math.cos(theta)
    y += delta_d * math.sin(theta)
    error = ThinkFunctions.compute_error(sensor_vals, method='binary')

    # ---------------------------- Think ---------------------------- #

    if state == 'IDLE':
        left_Speed = base_speed
        right_Speed = base_speed
        if current_task < len(pickup_nodes):
            state = 'IDLE'

    elif state == 'Line_following':
        print("Following line...")
        # 1. Object Detection
        if distance_mm <= recognition_distance:  # Check if distance is below threshold
            state = 'AVOID_OBSTACLE'

        # 2. Line Following PID Control
        correction = ThinkFunctions.pid_update(error, Kp=1.0, Ki=0.0, Kd=0.1)
        left_speed = base_speed - correction
        
        right_speed = base_speed + correction

        # 3. Node Detection
        num_active = sum(sensor_vals)
        if num_active >= 3:  # Three or more sensors active => cross section
            state = 'AT_NODE'  # Transition to whatever handles node logic

    elif state == 'AVOID_OBSTACLE':
        state_entry_time = ticks_ms()
        if ticks_diff(ticks_ms(), state_entry_time) < 1200:
            # During turn phase (1.2s)
            left_Speed = -base_speed
            right_Speed = base_speed
        else:
            left_Speed = 0
            right_Speed = 0
            returning_to_node = True
            state = 'Line_following'
            state_entry_time = ticks_ms()

    elif state == 'AT_NODE':
        if not path:
            print("No path left.")
            state = 'IDLE'

        prev_node = current_node
        current_node = path.pop(0)

        if not path:
            print("Reached goal node:", current_node)
            state = 'IDLE'

        next_node = path[0]
        turn = ThinkFunctions.get_turn_direction(prev_node, current_node, next_node)

        print(f"From {prev_node} to {current_node} → {next_node}, turn: {turn}")
        state = turn  # 'turn_left', 'turn_right', or 'go_straight'
        state_entry_time = ticks_ms()  # for timing motor action

    elif state == 'turn_left':
        if ticks_diff(ticks_ms(), state_entry_time) < 700:  # adjust timing for your robot
            left_Speed = -base_speed
            right_Speed = base_speed
        else:
            left_Speed = 0
            right_Speed = 0
            state = 'Line_following'
            state_entry_time = ticks_ms()

    elif state == 'turn_right':
        if ticks_diff(ticks_ms(), state_entry_time) < 700:
            left_Speed = base_speed
            right_Speed = -base_speed
        else:
            left_Speed = 0
            right_Speed = 0
            state = 'Line_following'
            state_entry_time = ticks_ms()
        

         
    # ----------------------------- Act ----------------------------- #
    print(f"state: {state}")
    print(f"Pose: x={x:.2f} cm, y={y:.2f} cm, θ={math.degrees(theta):.2f}°")
    print(f"distance: {distance_mm:.2f}")
    motorA.set_speed(left_Speed)
    motorB.set_speed(right_Speed)

    print(sensor_vals)
    print(counter)
    counter += 1
    sleep(0.1)