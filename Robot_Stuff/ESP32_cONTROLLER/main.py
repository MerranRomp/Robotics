from machine import Pin, I2C, PWM
from time import sleep, ticks_ms, ticks_diff
from Utils import SeeFunctions, ThinkFunctions, ActFunctions, nodes
from Utils.encoder_handler import setup_encoders, read_and_reset_ticks
import math

sleep(1)  # Short delay to allow hardware to stabilize

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

limit_switch = Pin(23, Pin.IN, Pin.PULL_UP)  # Assuming pin 23 and active LOW
# ------------------------ I2C & Motor Setup ------------------------ #
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
motorA = Motor(27, 14)
motorB = Motor(26, 25)
left_Speed = 0
right_Speed = 0

magnet_status = False
limit_triggered = False
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
WHEEL_DIAMETER_CM = 6.2
WHEEL_BASE_CM = 15.5
recognition_distance = 100  # mm
last_error = 0
line_lost_time = None
line_lost_duration = 3000  # milliseconds to keep applying last known correction
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
pickup_pairs = [('B1', 'A1'), ('B2', 'A2'), ('B3', 'A3'), ('B4', 'A4')]
dropoff_pairs = [('F9', 'G9'), ('F8', 'G8'), ('F7', 'G7'), ('F6', 'G6')]
current_box_index = 0

start = 'D1'
goal = 'A1'
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
    # See
    if True:
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
        pattern = sensor_vals
        if distance_mm <= recognition_distance:
            state = 'turn_left'
            
        total = sum(sensor_vals)

        if pattern == [0, 0, 0, 0, 0] or pattern == [1, 0, 0, 0, 0] or pattern == [0, 0, 0, 0, 1] or pattern == [0, 0, 0, 1, 1]  or pattern == [1, 1, 0, 0, 0]:
            print(f"node!: {sensor_vals}")
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
        pickup_node, pickup_target = pickup_pairs[current_box_index]
        dropoff_node, dropoff_target = dropoff_pairs[current_box_index]

        if ticks_diff(ticks_ms(), last_node_time) < 2000:
            print("Recently visited node — returning to Line_following")
            state = 'Line_following'
            continue

        last_node_time = ticks_ms()

        # Progress to next node first!
        prev_node = current_node
        if path:
            current_node = path.pop(0)
            print(f"At node: {current_node}")
        else:
            print("Warning: path empty in AT_NODE state!")
            state = 'stop'
            continue

        # Handle arrival at pickup or dropoff node
        if current_node == pickup_node:
            print(f"Arrived at pickup node: {current_node}")
            state = 'pick_up_box'
            state_entry_time = ticks_ms()
            continue

        elif current_node == dropoff_node:
            print(f"Arrived at drop-off node: {current_node}")
            state = 'drop_off_box'
            state_entry_time = ticks_ms()
            continue

        # Continue with path navi


    elif state == 'turn_left':
        if turn_start_angle is None:
            turn_start_angle = theta  # Record starting angle once

        angle_turned = ThinkFunctions.angle_difference(theta, turn_start_angle)

        if angle_turned < math.pi / 2:
            left_Speed = -base_speed_left
            right_Speed = base_speed_right
        else:
            # Start driving forward for 1 second
            left_Speed = base_speed_left
            right_Speed = base_speed_right
            turn_start_angle = None
            state = 'drive_after_turn'
            state_entry_time = ticks_ms()

    elif state == 'turn_right':
        if turn_start_angle is None:
            turn_start_angle = theta  # Record starting angle once

        angle_turned = ThinkFunctions.angle_difference(theta, turn_start_angle)

        if angle_turned < math.pi / 2:
            left_Speed = base_speed_left
            right_Speed = -base_speed_right
        else:
            # Start driving forward for 1 second
            left_Speed = base_speed_left
            right_Speed = base_speed_right
            turn_start_angle = None
            state = 'drive_after_turn'
            state_entry_time = ticks_ms()

    elif state == 'stop':
        left_Speed = 0
        right_Speed = 0

    elif state == 'drive_after_turn':

        if ticks_ms() - state_entry_time < 1000:
            # Drive straight
            left_Speed = base_speed_left
            right_Speed = base_speed_right
        else:
            # After 1s, resume line following
            left_Speed = 0
            right_Speed = 0
            state = 'Line_following'
            state_entry_time = ticks_ms()
            continue  # Skip current iteration to let next loop handle speeds
    
    elif state == 'pick_up_box':
        try:
            # Turn magnet on once
            if not magnet_status:
                print("Activating magnet")
                magnet_status = True
                magnet_pin.value(1)
                pickup_start_time = ticks_ms()

            # Drive forward
            left_Speed = base_speed_left
            right_Speed = base_speed_right

            # Poll switch
            if not limit_switch.value():  # Active LOW = pressed
                if limit_hold_start is None:
                    limit_hold_start = ticks_ms()
                elif ticks_diff(ticks_ms(), limit_hold_start) >= 500:
                    print("Limit switch held ≥ 0.5s — box picked up.")
                    limit_triggered = True
            else:
                limit_hold_start = None  # Reset if released too early

            # Transition if box is picked up
            if limit_triggered:
                left_Speed = 0
                right_Speed = 0
                state = 'pickup_complete'
                state_entry_time = ticks_ms()

        except Exception as e:
            print("ERROR in pick_up_box:", e)
            # Optional: go to safe state
            left_Speed = 0
            right_Speed = 0
            state = 'stop'
            
    elif state == 'pickup_complete':
        # Drive backwards slowly
        left_Speed = -base_speed_left
        right_Speed = -base_speed_right

        # Detect node pattern while reversing
        pattern = sensor_vals
        if pattern in ([1, 0, 0, 0, 0], [0, 0, 0, 0, 1], [0, 0, 0, 1, 1], [1, 1, 0, 0, 0], [1, 1, 1, 1, 1]):
            print("Node detected while reversing — switching to pathfinding")
            left_Speed = 0
            right_Speed = 0

            # Decide next destination (e.g., dropoff)
            pickup_node, pickup_target = pickup_pairs[current_box_index]
            dropoff_node, dropoff_target = dropoff_pairs[current_box_index]

            # Compute path from current node to drop-off
            path, cost = ThinkFunctions.dijkstra(pickup_target, dropoff_node)
            print("New path:", path)
            current_node = pickup_target  # Pretend we landed there after reverse
            if path and path[0] == current_node:
                path.pop(0)
            state = 'Line_following'
            state_entry_time = ticks_ms()



    
    # -------- Act -------- #
    if counter > 20:
        #print(f"speed: {left_Speed}  ,  {right_Speed}")
        print(f"State: {state}")
        #print(f"Pose: x={x:.2f} cm, y={y:.2f} cm, θ={math.degrees(theta):.2f}°")
        #print(f"Distance: {distance_mm:.2f} mm")
        #print(f"{sensor_vals}")
        counter = 0

    motorA.set_speed(left_Speed)
    motorB.set_speed(right_Speed)

    counter += 1
    sleep(0.01)
