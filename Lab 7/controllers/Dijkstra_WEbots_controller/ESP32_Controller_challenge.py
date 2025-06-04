from machine import Pin, UART
from time import sleep
from math import sqrt, atan2, pi

# ----------------- Dijkstra and Graph Setup -----------------

graph = {
    'A1': [('A2', 1), ('C1', 2.5)],
    'A2': [('A1', 1), ('A3', 1)],
    'A3': [('A2', 1), ('A4', 1)],
    'A4': [('A3', 1), ('A5', 2)],
    'A5': [('A4', 2), ('B5', 1.5), ('A9', 5)],
    'A9': [('A5', 5), ('B9', 1.5)],

    'B5': [('A5', 1.5), ('B9', 5), ('C5', 1)],
    'B9': [('A9', 1.5), ('B5', 5), ('C9', 1)],

    'C1': [('D1', 1), ('C5', 5), ('A1', 2.5)],
    'C5': [('C1', 5), ('B5', 1), ('C9', 5), ('D5', 1)],
    'C9': [('B9', 1), ('C5', 5), ('E9', 2.5)],

    'D1': [('C1', 1), ('D5', 5), ('E1', 1.5)],
    'D5': [('D1', 5), ('C5', 1), ('E5', 1.5)],

    'E1': [('D1', 1.5), ('E5', 5)],
    'E5': [('E1', 5), ('D5', 1.5), ('E6', 2)],
    'E6': [('E5', 2), ('E7', 1)],
    'E7': [('E6', 1), ('E8', 1)],
    'E8': [('E7', 1), ('E9', 1)],
    'E9': [('E8', 1), ('C9', 2.5)],
}

node_coords = {
    'A1': (-0.50, 0.25), 'A2': (-0.40, 0.25), 'A3': (-0.30, 0.25), 'A4': (-0.20, 0.25), 'A5': (0.00, 0.25),
    'A9': (0.50, 0.25), 'B5': (0.00, 0.10), 'B9': (0.50, 0.10), 'C1': (-0.50, 0.00), 'C5': (0.00, 0.00),
    'C9': (0.50, 0.00), 'D1': (-0.50, -0.10), 'D5': (0.00, -0.10), 'E1': (-0.50, -0.25), 'E5': (0.00, -0.25),
    'E6': (0.20, -0.25), 'E7': (0.30, -0.25), 'E8': (0.40, -0.25), 'E9': (0.50, -0.25)
}
from machine import Pin, UART
from time import sleep
from math import sqrt, atan2, pi

# ----------------- Dijkstra and Graph Setup -----------------

# (Unchanged graph and node_coords...)

# Functions unchanged

def distance(p1, p2): # Calculate Euclidean distance between two points
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return (dx**2 + dy**2) ** 0.5

def normalize_angle(angle): # Normalize angle to the range [-pi, pi]
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle

def dijkstra(graph, start):# Dijkstra's algorithm to find shortest paths from start node
    unvisited = set(graph.keys())
    distances = {node: float('inf') for node in graph}
    previous = {node: None for node in graph}
    distances[start] = 0

    while unvisited:
        current = min(unvisited, key=lambda node: distances[node])
        unvisited.remove(current)

        for neighbor, weight in graph[current]:
            alt = distances[current] + weight
            if alt < distances[neighbor]:
                distances[neighbor] = alt
                previous[neighbor] = current

    return distances, previous

def shortest_path(graph, start, end):# Find the shortest path from start to end node using Dijkstra's algorithm
    _, previous = dijkstra(graph, start)
    path = []
    node = end
    while node:
        path.insert(0, node)
        node = previous[node]
    return path



# ----------------- Hardware Setup -----------------

led_board = Pin(2, Pin.OUT)
led_yellow = Pin(4, Pin.OUT)
led_blue = Pin(23, Pin.OUT)
led_green = Pin(22, Pin.OUT)
led_red = Pin(21, Pin.OUT)
button_left = Pin(34, Pin.IN, Pin.PULL_DOWN)
button_right = Pin(35, Pin.IN, Pin.PULL_DOWN)

#start button sequence
print("Click the button on the ESP32 to continue. Then, close Thonny and run the Webots simulation.")
while button_left() == False:
    sleep(0.25)
    led_board.value(not led_board())

# Initialize UART for communication with the robot
uart = UART(1, 115200, tx=1, rx=3)
uart2 = UART(2, baudrate=115200, tx=17, rx=16)

# Initialize line sensors with default values
line_left = False
line_center = False
line_right = False

current_state = 'forward' # Initial state of the robot
counter = 0 # Counter for turning states
COUNTER_MAX = 5 # Maximum counter for turning states
COUNTER_STOP = 50 # Maximum counter for turning states
state_updated = True # Whether the state has been updated for sending to the robot
position_valid = False # Whether the robot's position is valid
turning_to_next_node = False # Whether the robot is currently turning to face the next node
turn_target_angle = 0.0 # Angle to turn to face the next node
path_index = 0 # Index of the current target node in the planned path
NAV_TOLERANCE = 0.02 # Tolerance for reaching a node

# ----------------- Path Planning -----------------
planned_path = shortest_path(graph, 'A1', 'E9')
print('Planned path:', planned_path)


# ----------------- Main Loop -----------------
while True:
    try:
        # -------- See --------
        #Read uart for incoming messages
        if uart.any():
            try:
                msg_bytes = uart.readline() # Read a line from UART
                msg_str = msg_bytes.decode('utf-8').strip() # Decode the message
                parts = msg_str.split(',') # Split the message into parts

                line_bits = parts[0] # First part is the line sensor bits
                if len(line_bits) != 3:
                    raise ValueError("Invalid sensor bits")

                current_x = float(parts[1]) # Second part is the current X position
                current_y = float(parts[2]) # Third part is the current Y position
                current_yaw = float(parts[3]) # Fourth part is the current yaw angle
                position_valid = True

                line_left = line_bits[0] == '1'
                line_center = line_bits[1] == '1'
                line_right = line_bits[2] == '1'

                led_blue.value(line_left)
                led_green.value(line_center)
                led_red.value(line_right)

                uart2.write(current_state + '\n')
                uart2.write(f"[Pos] X: {current_x:.2f}, Y: {current_y:.2f}, Yaw: {current_yaw:.2f}\n".encode())

            except Exception as e:
                print("UART parse error:", e, "| Raw:", msg_bytes)
                continue
        # -------- Think --------
        # -------- Navigation Logic --------
        # Box of black magic
        if position_valid and path_index < len(planned_path):
            current_target = planned_path[path_index]
            target_coords = node_coords[current_target]
            dist = distance((current_x, current_y), target_coords)

            uart2.write(f"[NodeCheck] Target: {current_target} | Pos: ({current_x:.2f}, {current_y:.2f}) | Dist: {dist:.3f}\n".encode())

            if dist < NAV_TOLERANCE:
                uart2.write(f"[Nav] Reached {current_target}\n".encode())

                if path_index + 1 < len(planned_path):
                    next_target = planned_path[path_index + 1]
                    from_coords = node_coords[current_target]
                    to_coords = node_coords[next_target]
                    dx = to_coords[0] - from_coords[0]
                    dy = to_coords[1] - from_coords[1]
                    turn_target_angle = atan2(dy, dx)
                    current_state = 'align'
                    state_updated = True
                    uart2.write(f"[Turn] Preparing to face {next_target}\n".encode())
                else:
                    uart2.write("[Nav] All nodes reached. Stopping.\n".encode())
                    current_state = 'stop'
                    state_updated = True

        #state machine logic
        if current_state == 'forward':
            counter = 0
            if line_right and not line_left:
                current_state = 'turn_right'
                state_updated = True
            elif line_left and not line_right:
                current_state = 'turn_left'
                state_updated = True
            elif button_right.value():
                current_state = 'stop'
                state_updated = True

        elif current_state == 'turn_right':
            if counter >= 3:
                current_state = 'forward'
                state_updated = True
            elif button_right.value():
                current_state = 'stop'
                state_updated = True

        elif current_state == 'turn_left':
            if counter >= 3:
                current_state = 'forward'
                state_updated = True
            elif button_right.value():
                current_state = 'stop'
                state_updated = True


        # -------- Align Logic --------
        elif current_state == 'align':
            yaw_error = normalize_angle(turn_target_angle - current_yaw)
            uart2.write(f"[Align] Target: {turn_target_angle:.2f}, Yaw: {current_yaw:.2f}, Err: {yaw_error:.2f}\n".encode())

            if abs(yaw_error) < 0.1:
                path_index += 1
                current_state = 'forward'
                state_updated = True
                uart2.write("[Align] Done. Resuming forward\n".encode())
            elif abs(yaw_error) < 0.25:
                current_state = 'forward'
                state_updated = True
                uart2.write("[Align] Close enough. Stopping to reduce overshoot\n".encode())
            else:
                current_state = 'turn_left' if yaw_error > 0 else 'turn_right'
                state_updated = True

        # -------- Act --------
        #send state over uart
        if state_updated:
            uart.write(current_state + '\n')
            state_updated = False

        counter += 1
        sleep(0.02)

    except Exception as e:
        print("Loop error:", e)



