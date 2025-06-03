from machine import Pin, UART
from time import sleep
from math import sqrt

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
    'A1': (-0.50, 0.25),
    'A2': (-0.40, 0.25),
    'A3': (-0.30, 0.25),
    'A4': (-0.20, 0.25),
    'A5': (0.00, 0.25),
    'A9': (0.50, 0.25),
    'B5': (0.00, 0.10),
    'B9': (0.50, 0.10),
    'C1': (-0.50, 0.00),
    'C5': (0.00, 0.00),
    'C9': (0.50, 0.00),
    'D1': (-0.50, -0.10),
    'D5': (0.00, -0.10),
    'E1': (-0.50, -0.25),
    'E5': (0.00, -0.25),
    'E6': (0.20, -0.25),
    'E7': (0.30, -0.25),
    'E8': (0.40, -0.25),
    'E9': (0.50, -0.25)
}

def distance(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def dijkstra(graph, start):
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

def shortest_path(graph, start, end):
    _, previous = dijkstra(graph, start)
    path = []
    node = end
    while node:
        path.insert(0, node)
        node = previous[node]
    return path

planned_path = shortest_path(graph, 'A1', 'D5')
print('Planned path:', planned_path)
path_index = 0  # Start at first node in the path
NAV_TOLERANCE = 0.05  # 5 cm tolerance (adjust as needed)

# ----------------- Hardware Setup -----------------

led_board = Pin(2, Pin.OUT)
led_yellow = Pin(4, Pin.OUT)
led_blue = Pin(23, Pin.OUT)
led_green = Pin(22, Pin.OUT)
led_red = Pin(21, Pin.OUT)
button_left = Pin(34, Pin.IN, Pin.PULL_DOWN)
button_right = Pin(35, Pin.IN, Pin.PULL_DOWN)

print("Click the button on the ESP32 to continue. Then, close Thonny and run the Webots simulation.")
while button_left() == False:
    sleep(0.25)
    led_board.value(not led_board())

uart = UART(1, 115200, tx=1, rx=3)


line_left = False
line_center = False
line_right = False

current_state = 'forward'
counter = 0
COUNTER_MAX = 5
COUNTER_STOP = 50
state_updated = True

while True:
    # -------- See --------
    if uart.any():
        try:
            msg_bytes = uart.readline()
            msg_str = msg_bytes.decode('utf-8').strip()

            parts = msg_str.split(',')

            line_bits = parts[0]
            if len(line_bits) != 3:
                raise ValueError("Invalid sensor bits")

            current_x = float(parts[1])
            current_y = float(parts[2])
            current_yaw = float(parts[3])
            

            line_left = line_bits[0] == '1'
            line_center = line_bits[1] == '1'
            line_right = line_bits[2] == '1'

            # LED feedback for debugging
            led_blue.value(line_left)
            led_green.value(line_center)
            led_red.value(line_right)

        except Exception as e:
            print("UART parse error:", e, "| Raw:", msg_bytes)
            continue
    
    
    #-----------Think --------
    if current_state == 'forward':
        counter = 0
        if line_right and not line_left:
            current_state = 'turn_right'
            state_updated = True
        elif line_left and not line_right:
            current_state = 'turn_left'
            state_updated = True
        elif line_left and line_right and line_center:
            current_state = 'turn_left'
            state_updated = True
        elif line_left and line_center and not line_right:
            current_state = 'forward'
            state_updated = True
        elif button_right.value() == True:
            current_state = 'stop'
            state_updated = True

    elif current_state == 'turn_right':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
        elif button_right.value() == True:
            current_state = 'stop'
            state_updated = True

    elif current_state == 'turn_left':
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
        elif button_right.value() == True:
            current_state = 'stop'
            state_updated = True

    elif current_state == 'stop':
        led_board.value(1)
        if counter >= COUNTER_STOP:
            current_state = 'forward'
            state_updated = True
            led_board.value(0)

    # -------- Act --------
    if state_updated == True:
        uart.write(current_state + '\n')
        state_updated = False

    counter += 1
    sleep(0.02)


