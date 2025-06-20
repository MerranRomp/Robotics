from machine import Pin
from time import sleep, ticks_ms, ticks_diff
from Utils import SeeFunctions, ActFunctions
import math
import nodes
import network
import socket
import time

from Utils import ThinkFunctions
print("Loaded from:", ThinkFunctions.__file__)
print("Contents:", dir(ThinkFunctions))

# ------------------------- WiFi Setup ------------------------- #
ssid = 'RobotNet'
# Disable AP mode to avoid conflicts
ap = network.WLAN(network.AP_IF)
ap.active(False)

# Reset and activate STA mode
sta = network.WLAN(network.STA_IF)
sta.active(False)
time.sleep(1)
sta.active(True)
time.sleep(1)

# Optional: Assign static IP to avoid DHCP issues
sta.ifconfig(('192.168.4.20', '255.255.255.0', '192.168.4.1', '192.168.4.1'))

# Connect to open AP (no password)
print("Connecting to RobotNet...")
sta.connect(ssid)

# Wait for connection (10 seconds max)
timeout = 10
while not sta.isconnected() and timeout > 0:
    print("Waiting for connection...")
    time.sleep(1)
    timeout -= 1

if not sta.isconnected():
    print("Failed to connect to RobotNet")
else:
    print("Connected:", sta.ifconfig())

    # Setup UDP
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


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

IR_sensor_pins = [36, 34, 35, 32, 39]
counter = 0
COUNTER_MAX = 5
COUNTER_STOP = 50

current_state = 'Line_following'
object_detected = False
state_updated = True
left_Speed = 0
right_Speed = 0
base_speed = 50  # Base speed in %
returning_to_node = False
state_entry_time = ticks_ms()

#statemachine variables
pickup_nodes = ['A1', 'A2', 'A3', 'A4']
dropoff_nodes = ['G6', 'G7', 'G8', 'G9']
start = 'A1'
goal = 'F9'
state = 'Line_following'
path = []
current_task = 0
last_node = None

path, cost = ThinkFunctions.dijkstra(start, goal)
print("Path:", path)
print("Total cost:", cost)



#robot starts at the first node
current_node = 'F1'

# --------------------------- Initialization --------------------------- #
SeeFunctions.setup_ir_sensors(*IR_sensor_pins)
SeeFunctions.setup_VL53L0X()
ActFunctions.motor_setup(26, 27)

# ----------------------------- Main Loop ----------------------------- #
while True:
    # ----------------------------- See ----------------------------- #
    distance_mm = SeeFunctions.TOFdistance() 
    sensor_vals = SeeFunctions.read_binary_values()
    print(sensor_vals)
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
    print(error)

    # ---------------------------- Think ---------------------------- #

    if state == 'IDLE':
        if current_task < len(pickup_nodes):
            state = 'PLAN_PATH'

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
        

    ActFunctions.motor_speed(left_Speed, right_Speed)     
    #msg = f"distance: {distance_mm:.2f} x: {x:.2f} y: {y:.2f} theta: {theta:.2f}"
    #udp.sendto(msg.encode(), ('192.168.4.1', 1234))  # Send to receiver AP
    #print("Sent:", msg)
    print(f"Pose: x={x:.2f} cm, y={y:.2f} cm, θ={math.degrees(theta):.2f}°")
    print(f"distance: {distance_mm:.2f}")
    print(state)
    ActFunctions.motor_speed(left_Speed, right_Speed)
    print(counter)
    counter += 1
    sleep(0.1)
