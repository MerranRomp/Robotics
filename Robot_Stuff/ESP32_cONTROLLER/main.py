from machine import Pin, I2C, PWM
from time import sleep, ticks_ms, ticks_diff
from Utils import SeeFunctions, ThinkFunctions, ActFunctions, nodes
from Utils.encoder_handler import setup_encoders, read_and_reset_ticks
import math

sleep(1)  # Short delay to allow hardware to stabilize

# ------------------------ Limit Switch Setup ------------------------ #


# ------------------------ Motor Class ------------------------ #
class Motor:
    def __init__(self, pin_fwd, pin_rev, freq=1000, min_effective=35):
        self.pwm_fwd = PWM(Pin(pin_fwd))
        self.pwm_fwd.freq(freq)
        self.pwm_rev = PWM(Pin(pin_rev))
        self.pwm_rev.freq(freq)
        self.min_effective = min_effective  # Minimum duty% to actually move the motor

    def set_speed(self, speed_percent):
        speed_percent = max(min(speed_percent, 100), -100)

        # Normalize to avoid dead zone
        if abs(speed_percent) > 0:
            sign = 1 if speed_percent > 0 else -1
            # Scale to range [min_effective, 100]
            scaled_speed = (abs(speed_percent) / 100) * (100 - self.min_effective) + self.min_effective
            duty = int((scaled_speed / 100) * 65535)
            if sign > 0:
                self.pwm_fwd.duty_u16(duty)
                self.pwm_rev.duty_u16(0)
            else:
                self.pwm_fwd.duty_u16(0)
                self.pwm_rev.duty_u16(duty)
        else:
            self.stop()

    def stop(self):
        self.pwm_fwd.duty_u16(0)
        self.pwm_rev.duty_u16(0)

        
def handle_limit_switch(pin):
    global magnet_status
    #magnet_status = not magnet_status  # Toggle state
    #magnet_pin.value(magnet_status)    # Set output pin accordingly
    
    print("Limit switch pressed!")
    

SeeFunctions.setup_button(pin_number=23, callback=handle_limit_switch)
# ------------------------ I2C & Motor Setup ------------------------ #
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
motorA = Motor(27, 14)
motorB = Motor(26, 25)
left_Speed = 0
right_Speed = 0

magnet_status = 0
magnet_status = False
magnet_pin = Pin(16, Pin.OUT)

setup_encoders({
    'a1': 18,
    'b1': 13,
    'a2': 12,
    'b2': 5
})

# ------------------------ Constants & State ------------------------ #
PPR = 16
GEAR_RATIO = 100
WHEEL_DIAMETER_CM = 6.4
WHEEL_BASE_CM = 15.5
recognition_distance = 100  # mm
last_error = 0
line_lost_time = None
line_lost_duration = 5000  # milliseconds to keep applying last known correction
last_node_time = 0  # time of last AT_NODE trigger


x, y, theta = 0.0, 0.0, 0.0  # Initial pose
IR_sensor_pins = [35, 34, 36, 32, 39]
counter = 0
base_speed_left = (50)
base_speed_right = (49)
returning_to_node = False
state_entry_time = ticks_ms()
turn_start_angle = None

# State Machine
pickup_nodes = ['A1', 'A2', 'A3', 'A4']
dropoff_nodes = ['G6', 'G7', 'G8', 'G9']
start = 'F1'
goal = 'D1'
state = 'IDLE'
path, cost = ThinkFunctions.dijkstra(start, goal)
if path and path[0] == start:
    print(f"Removing starting node '{start}' from path")
    path.pop(0)
    
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
        state = 'Line_following'
        
    elif state == 'Line_following':
        if distance_mm <= recognition_distance:
            state = 'turn_left'
            
        total = sum(sensor_vals)

        if total <= 2:
            state = 'AT_NODE'

        elif total == 5:
            # Line lost
            if last_error < 0:
                left_Speed = base_speed_left
                right_Speed = base_speed_right * 0.3
            elif last_error > 0:
                left_Speed = base_speed_left * 0.3
                right_Speed = base_speed_right
            else:
                left_Speed = -base_speed_left
                right_Speed = base_speed_right

        else:
            pattern = sensor_vals

            # Centered
            if pattern == [1, 1, 0, 1, 1] or pattern == [1, 0, 0, 0, 1] or pattern == [0, 1, 0, 1, 0]:
                left_Speed = base_speed_left
                right_Speed = base_speed_right

            # Slight left
            elif pattern == [1, 0, 0, 1, 1] or pattern == [1, 0, 1, 1, 1] or pattern == [1, 0, 0, 0, 1] or pattern == [0, 1, 0, 1, 1] or pattern == [0, 1, 0, 0, 1]:
                left_Speed = base_speed_left * 0.8
                right_Speed = base_speed_right * 1.1

            # Slight right
            elif pattern == [1, 1, 0, 0, 1] or pattern == [1, 1, 1, 0, 1] or pattern == [1, 0, 0, 0, 1] or pattern == [1, 1, 0, 1, 0] or pattern == [1, 0, 0, 1, 0]:
                left_Speed = base_speed_left * 1.1
                right_Speed = base_speed_right * 0.8

            # Hard left
            elif pattern == [0, 1, 1, 1, 1] or pattern == [0, 0, 1, 1, 1] or pattern == [0, 1, 1, 0, 1] or pattern == [0, 1, 0, 0, 1] or pattern == [0, 1, 0, 0, 0] or pattern == [0, 1, 1, 0, 0]:
                left_Speed = base_speed_left * 0.6
                right_Speed = base_speed_right * 1.2

            # Hard right
            elif pattern == [1, 1, 1, 1, 0] or pattern == [1, 1, 1, 0, 0] or pattern == [1, 0, 1, 0, 0] or pattern == [1, 0, 0, 0, 0] or pattern == [1, 0, 0, 1, 0]:
                left_Speed = base_speed_left * 1.2
                right_Speed = base_speed_right * 0.6

            else:
                # Fallback
                left_Speed = base_speed_left
                right_Speed = base_speed_right

                    
        
    elif state == 'AVOID_OBSTACLE':
        if ticks_diff(ticks_ms(), state_entry_time) < 5000:  # e.g. 2300 for 90-degree turn
            left_Speed = -base_speed_left
            right_Speed = base_speed_right
        else:
            left_Speed = 0
            right_Speed = 0
            state = 'Line_following'
            state_entry_time = ticks_ms()

    elif state == 'AT_NODE':
        print("At node:", current_node)

        # Prevent re-triggering if node was visited recently
        if ticks_diff(ticks_ms(), last_node_time) < 2000:
            print("Recently visited node — returning to Line_following")
            state = 'Line_following'

        else:
            last_node_time = ticks_ms()  # Update the last confirmed node time

            if path and path[0] == current_node:
                print(f"Removing duplicate current node '{current_node}' from path")
                path.pop(0)

            if not path:
                print("No path left. Going to IDLE.")
                state = 'IDLE'

            else:
                prev_node = current_node
                next_node = path[0]
                after_next_node = path[1] if len(path) > 1 else next_node

                directions = ThinkFunctions.get_turn_directions(nodes.graph, [prev_node, next_node, after_next_node])
                _, turn = directions[0]

                print(f"Turning {turn} from {prev_node} → {next_node} → {after_next_node}")

                current_node = path.pop(0)

                if turn == 'left':
                    state = 'turn_left'
                elif turn == 'right':
                    state = 'turn_right'
                else:
                    state = 'Line_following'

                state_entry_time = ticks_ms()






    elif state == 'turn_left':
        if turn_start_angle is None:
            turn_start_angle = theta  # Record starting angle once

        angle_turned = ThinkFunctions.angle_difference(theta, turn_start_angle)

        if angle_turned < math.pi / 2:
            left_Speed = -base_speed_left
            right_Speed = base_speed_right
        else:
            left_Speed = 0
            right_Speed = 0
            turn_start_angle = None
            state = 'Forward'
            state_entry_time = ticks_ms()


    elif state == 'turn_right':
        if turn_start_angle is None:
            turn_start_angle = theta

        angle_turned = ThinkFunctions.angle_difference(theta, turn_start_angle)

        if angle_turned < math.pi / 2:
            left_Speed = base_speed_left
            right_Speed = -base_speed_right
        else:
            left_Speed = 0
            right_Speed = 0
            turn_start_angle = None
            state = 'Forward'
            state_entry_time = ticks_ms()

    elif state == 'stop':
        left_Speed = 0
        right_Speed = 0

    # -------- Act -------- #
    if counter > 20:
        print(f"speed: {left_Speed}  ,  {right_Speed}")
        print(f"State: {state}")
        print(f"Pose: x={x:.2f} cm, y={y:.2f} cm, θ={math.degrees(theta):.2f}°")
        print(f"Distance: {distance_mm:.2f} mm")
        print(f"{sensor_vals}")
        counter = 0

    motorA.set_speed(left_Speed)
    motorB.set_speed(right_Speed)

    counter += 1
    sleep(0.02)