from machine import Pin, I2C, PWM
from time import sleep, ticks_ms, ticks_diff
from Utils import SeeFunctions, ThinkFunctions, ActFunctions
from Utils.encoder_handler import setup_encoders, read_and_reset_ticks
import math

sleep(1)  # Short delay to allow hardware to stabilize

# ------------------------ Limit Switch Setup ------------------------ #


# ------------------------ Motor Class ------------------------ #
class Motor:
    def __init__(self, pin_fwd, pin_rev, freq=1000):
        self.pwm_fwd = PWM(Pin(pin_fwd))
        self.pwm_fwd.freq(freq)
        self.pwm_rev = PWM(Pin(pin_rev))
        self.pwm_rev.freq(freq)

    def set_speed(self, speed_percent):
        speed_percent = max(min(speed_percent, 100), -100)
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
def handle_limit_switch(pin):
    print("Limit switch pressed!")

SeeFunctions.setup_button(pin_number=4, callback=handle_limit_switch)
# ------------------------ I2C & Motor Setup ------------------------ #
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
motorA = Motor(27, 14)
motorB = Motor(25, 26)

setup_encoders({
    'a1': 18,
    'b1': 13,
    'a2': 12,
    'b2': 5
})

# ------------------------ Constants & State ------------------------ #
PPR = 16
GEAR_RATIO = 120
WHEEL_DIAMETER_CM = 6.5
WHEEL_BASE_CM = 15.5
recognition_distance = 200  # mm

x, y, theta = 0.0, 0.0, 0.0  # Initial pose
IR_sensor_pins = [36, 34, 35, 32, 39]
counter = 0
base_speed_left = 50
base_speed_right = 53.5
returning_to_node = False
state_entry_time = ticks_ms()

# State Machine
pickup_nodes = ['A1', 'A2', 'A3', 'A4']
dropoff_nodes = ['G6', 'G7', 'G8', 'G9']
start = 'F1'
goal = 'D5'
state = 'IDLE'
path, cost = ThinkFunctions.dijkstra(start, goal)
current_task = 0
current_node = 'F1'
print("Path:", path)
print("Total cost:", cost)

# ------------------------ Sensor Initialization ------------------------ #
print("Init sensors")
SeeFunctions.setup_ir_sensors(*IR_sensor_pins)
SeeFunctions.setup_VL53L0X(i2c)

# ------------------------ Main Loop ------------------------ #
last_update = ticks_ms()

while True:
    current_time = ticks_ms()
    dt = ticks_diff(current_time, last_update)
    last_update = current_time

    # -------- See -------- #
    distance_mm = SeeFunctions.TOFdistance()
    sensor_vals = SeeFunctions.read_binary_values()

    # Read encoder deltas and reset counts
    delta_ticks_left, delta_ticks_right = read_and_reset_ticks()

    # Calculate wheel speeds and pose update
    left_distance_cm = (delta_ticks_left / PPR / GEAR_RATIO) * math.pi * WHEEL_DIAMETER_CM
    right_distance_cm = (delta_ticks_right / PPR / GEAR_RATIO) * math.pi * WHEEL_DIAMETER_CM
    x, y, theta = ThinkFunctions.update_pose(x, y, theta, left_distance_cm, right_distance_cm, WHEEL_BASE_CM)
    error = ThinkFunctions.compute_error(sensor_vals, method='binary')

    # -------- Think -------- #
    if state == 'IDLE':
        left_Speed = base_speed_left
        right_Speed = base_speed_right

    elif state == 'Line_following':
        if distance_mm <= recognition_distance:
            state = 'AVOID_OBSTACLE'

        correction = ThinkFunctions.pid_update(error, Kp=1.0, Ki=0.0, Kd=0.1)
        left_Speed = base_speed_left - correction
        right_Speed = base_speed_right + correction

        if sum(sensor_vals) >= 3:
            state = 'AT_NODE'

    elif state == 'AVOID_OBSTACLE':
        if ticks_diff(ticks_ms(), state_entry_time) < 1200:
            left_Speed = -base_speed_left
            right_Speed = base_speed_right
        else:
            left_Speed = 0
            right_Speed = 0
            state = 'Line_following'
            state_entry_time = ticks_ms()

    elif state == 'AT_NODE':
        if not path:
            state = 'IDLE'
        else:
            prev_node = current_node
            current_node = path.pop(0)
            if not path:
                print("Reached goal node:", current_node)
                state = 'IDLE'
            else:
                next_node = path[0]
                turn = ThinkFunctions.get_turn_direction(prev_node, current_node, next_node)
                print(f"From {prev_node} to {current_node} → {next_node}, turn: {turn}")
                state = turn
                state_entry_time = ticks_ms()

    elif state == 'turn_left':
        if ticks_diff(ticks_ms(), state_entry_time) < 700:
            left_Speed = -base_speed_left
            right_Speed = base_speed_right
        else:
            left_Speed = 0
            right_Speed = 0
            state = 'Line_following'
            state_entry_time = ticks_ms()

    elif state == 'turn_right':
        if ticks_diff(ticks_ms(), state_entry_time) < 700:
            left_Speed = base_speed_left
            right_Speed = -base_speed_right
        else:
            left_Speed = 0
            right_Speed = 0
            state = 'Line_following'
            state_entry_time = ticks_ms()

    # -------- Act -------- #
    if counter > 20:
        print(f"State: {state}")
        print(f"Pose: x={x:.2f} cm, y={y:.2f} cm, θ={math.degrees(theta):.2f}°")
        print(f"Distance: {distance_mm:.2f} mm")
        counter = 0

    motorA.set_speed(left_Speed)
    motorB.set_speed(right_Speed)

    counter += 1
    sleep(0.1)