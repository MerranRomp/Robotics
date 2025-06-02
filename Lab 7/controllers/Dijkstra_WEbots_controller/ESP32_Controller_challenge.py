from machine import Pin, UART
from time import sleep

# ----------------- Dijkstra and Graph Setup -----------------

graph = {
    'A1': [('A2', 1), ('C1', 1)],
    'A2': [('A1', 1), ('A3', 1)],
    'A3': [('A2', 1), ('A4', 1)],
    'A4': [('A3', 1), ('A5', 1)],
    'A5': [('A4', 1), ('B5', 1), ('A9', 1)],
    'A9': [('A5', 1), ('B1', 1)],

    'B1': [('A1', 1), ('C1', 1)],
    'B5': [('A5', 1), ('B9', 1), ('C5', 1)],
    'B9': [('A9', 1), ('B5', 1), ('C9', 1)],

    'C1': [('D1', 1), ('C5', 1), ('B1', 1)],
    'C5': [('C1', 1), ('B5', 1), ('C9', 1), ('D5', 1)],
    'C9': [('B9', 1), ('C5', 1), ('E9', 1)],

    'D1': [('C1', 1), ('D5', 1), ('E1', 1)],
    'D5': [('D1', 1), ('C5', 1), ('E5', 1)],

    'E1': [('D1', 1), ('E5', 1)],
    'E5': [('E1', 1), ('D5', 1), ('E6', 1)],
    'E6': [('E5', 1), ('E7', 1)],
    'E7': [('E6', 1), ('E8', 1)],
    'E8': [('E7', 1), ('E9', 1)],
    'E9': [('E8', 1), ('C9', 1)],
}


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

planned_path = shortest_path(graph, 'C1', 'D5')
print('Planned path:', planned_path)
current_index = 0  # start at first node in path

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

# --- NEW ---
node_detected = False
node_cooldown = 0

while True:
    # -------- See --------
    if uart.any():
        msg_bytes = uart.read()
        msg_str = str(msg_bytes, 'UTF-8')

        if msg_str[-4:-3] == '1':
            line_left = True
            led_blue.on()
        else:
            line_left = False
            led_blue.off()

        if msg_str[-3:-2] == '1':
            line_center = True
            led_green.on()
        else:
            line_center = False
            led_green.off()

        if msg_str[-2:-1] == '1':
            line_right = True
            led_red.on()
        else:
            line_right = False
            led_red.off()

    # --- NEW --- Node detection logic
    if not node_detected:
        if (line_left and line_center and line_right) or (not line_left and not line_center and not line_right):
            node_detected = True
            node_cooldown = 20  # ~0.4 sec at 20ms step
            print(">>> Node detected!")

    if node_cooldown > 0:
        node_cooldown -= 1
    else:
        node_detected = False

    # -------- Think --------
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

