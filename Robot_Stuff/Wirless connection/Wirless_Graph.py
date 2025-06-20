import matplotlib.pyplot as plt
import serial
import threading
import time

# Replace with your correct serial port and baud rate
SERIAL_PORT = 'COM5'  # e.g. '/dev/ttyUSB0' on Linux
BAUD_RATE = 115200

# Graph data (same as yours)
graph = {
    'A1': {'pos': (-5, 4), 'neighbors': {'B1': 1}},
    'A2': {'pos': (-4, 4), 'neighbors': {'B2': 1}},
    'A3': {'pos': (-3, 4), 'neighbors': {'B3': 1}},
    'A4': {'pos': (-2, 4), 'neighbors': {'B4': 1}},
    'B1': {'pos': (-5, 3), 'neighbors': {'A1': 1, 'B2': 1, 'D1': 2}},
    'B2': {'pos': (-4, 3), 'neighbors': {'A2': 1, 'B1': 1, 'B3': 1}},
    'B3': {'pos': (-3, 3), 'neighbors': {'A3': 1, 'B2': 1, 'B4': 1}},
    'B4': {'pos': (-2, 3), 'neighbors': {'A4': 1, 'B3': 4, 'B5': 2}},
    'B5': {'pos': (0, 3), 'neighbors': {'B4': 2, 'B9': 5, 'D5': 3}},
    'B9': {'pos': (5, 3), 'neighbors': {'B5': 5, 'C9': 2}},
    'C5': {'pos': (0, 1), 'neighbors': {'B5': 2, 'C9': 5, 'D5': 1}},
    'C9': {'pos': (5, 1), 'neighbors': {'C5': 5, 'B9': 2, 'D9': 1}},
    'D1': {'pos': (-5, 0), 'neighbors': {'B1': 3, 'D5': 5, 'E1': 1}},
    'D5': {'pos': (0, 0), 'neighbors': {'D1': 5, 'E5': 1, 'D9': 5, 'C5': 1}},
    'D9': {'pos': (5, 0), 'neighbors': {'D5': 5, 'C9': 1, 'F9': 3}},
    'E1': {'pos': (-5, -1), 'neighbors': {'D1': 1, 'F1': 2, 'E5': 5}},
    'E5': {'pos': (0, -1), 'neighbors': {'E1': 5, 'D5': 1, 'F5': 2}},
    'F1': {'pos': (-5, -3), 'neighbors': {'E1': 2, 'F5': 5}},
    'F5': {'pos': (0, -3), 'neighbors': {'F1': 5, 'E5': 2, 'F6': 2}},
    'F6': {'pos': (2, -3), 'neighbors': {'F5': 2, 'F7': 1, 'G6': 1}},
    'F7': {'pos': (3, -3), 'neighbors': {'F6': 1, 'F8': 1, 'G7': 1}},
    'F8': {'pos': (4, -3), 'neighbors': {'F7': 1, 'F9': 1, 'G8': 1}},
    'F9': {'pos': (5, -3), 'neighbors': {'F8': 1, 'G9': 1, 'D9': 3}},
    'G6': {'pos': (2, -4), 'neighbors': {'F6': 1}},
    'G7': {'pos': (3, -4), 'neighbors': {'F7': 1}},
    'G8': {'pos': (4, -4), 'neighbors': {'F8': 1}},
    'G9': {'pos': (5, -4), 'neighbors': {'F9': 1}},
}

# Global variable for the robot's current node
robot_node = None

def serial_reader():
    global robot_node
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}")
        
        while True:
            if ser.in_waiting:
                line = ser.readline().decode().strip()
                if line in graph:
                    robot_node = line
                    print(f"Received node: {robot_node}")
                else:
                    print(f"Ignored unknown node: {line}")
    except Exception as e:
        print("Serial error:", e)

# Start serial reader in the background
threading.Thread(target=serial_reader, daemon=True).start()

# Plot setup
plt.ion()
fig, ax = plt.subplots(figsize=(10, 7))

def draw_graph():
    ax.clear()
    
    for node, data in graph.items():
        x, y = data['pos']
        ax.plot(x, y, 'ko', zorder=3)  # black node
        ax.text(x, y + 0.2, node, ha='center', fontsize=8, zorder=4)
        
        for neighbor in data['neighbors']:
            if neighbor in graph:
                nx, ny = graph[neighbor]['pos']
                ax.plot([x, nx], [y, ny], color='black', linewidth=1.5, zorder=1)

    # Draw robot position
    if robot_node and robot_node in graph:
        rx, ry = graph[robot_node]['pos']
        ax.plot(rx, ry, 'ro', markersize=12, zorder=5)
        ax.text(rx, ry - 0.3, '🤖', ha='center', fontsize=12, zorder=6)

    ax.set_title("Graph with Live Robot Position")
    ax.grid(True)
    ax.set_aspect('equal')
    plt.pause(0.1)

# Continuous live update
while True:
    draw_graph()
    time.sleep(0.2)
