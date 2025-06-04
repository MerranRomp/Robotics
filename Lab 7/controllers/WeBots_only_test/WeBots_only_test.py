from controller import Robot, GPS, InertialUnit
import math
import matplotlib.pyplot as plt

# ---------- Graph with distances ----------
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

# ---------- Dijkstra ----------
def dijkstra(graph, start):
    unvisited = set(graph.keys())
    distances = {node: float('inf') for node in graph}
    previous = {node: None for node in graph}
    distances[start] = 0

    while unvisited:
        current = min(unvisited, key=lambda node: distances[node])
        if distances[current] == float('inf'):
            break
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

# ---------- Visualization ----------
def draw_map(path, current_pos=None):
    plt.clf()
    for node, neighbors in graph.items():
        for neighbor, _ in neighbors:
            x1, y1 = node_coords[node]
            x2, y2 = node_coords[neighbor]
            plt.plot([x1, x2], [y1, y2], 'gray', linewidth=0.5)

    for node, (x, y) in node_coords.items():
        plt.plot(x, y, 'ko', markersize=4)
        plt.text(x + 0.01, y + 0.01, node, fontsize=8)

    path_x = [node_coords[n][0] for n in path]
    path_y = [node_coords[n][1] for n in path]
    plt.plot(path_x, path_y, 'b--', linewidth=2, label='Planned Path')

    if current_pos:
        plt.plot(current_pos[0], current_pos[1], 'ro', markersize=8, label='Robot')

    plt.axis('equal')
    plt.title('Robot Navigation Path')
    plt.legend()
    plt.pause(0.001)

# ---------- Setup ----------
robot = Robot()
timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

gps = robot.getDevice('gps')
gps.enable(timestep)

inertial_unit = robot.getDevice('inertial unit')
inertial_unit.enable(timestep)

gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for name in gsNames:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    gs.append(sensor)

# ---------- Constants ----------
MAX_SPEED = 6.28
BASE_SPEED = 0.35 * MAX_SPEED
MIN_SPEED = 0.2 * MAX_SPEED
ROTATION_SPEED = 0.25 * MAX_SPEED
ANGLE_TOLERANCE = 0.15
DISTANCE_TOLERANCE = 0.035

# ---------- Navigation ----------
start_node = 'A1'
goal_node = 'B9'
path = shortest_path(graph, start_node, goal_node)
print("Planned path:", path)

current_target_index = 1
state = "TURNING"

# Enable interactive plotting
plt.ion()
draw_map(path)

# ---------- Helper Functions ----------
def get_heading():
    readings = []
    for _ in range(5):
        yaw = inertial_unit.getRollPitchYaw()[2]
        heading = yaw % (2 * math.pi)
        readings.append(heading)
        if robot.step(timestep) == -1: break
    return sum(readings) / len(readings)

def angle_to_target(pos, heading, target):
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    target_angle = math.atan2(dy, dx) % (2 * math.pi)
    angle_diff = target_angle - heading
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    return angle_diff

def distance_to_target(pos, target):
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    return math.sqrt(dx * dx + dy * dy)

def line_following(distance_to_node):
    vals = [sensor.getValue() for sensor in gs]
    threshold = 500
    left = vals[0] < threshold
    center = vals[1] < threshold
    right = vals[2] < threshold

    scale = min(1.0, distance_to_node / 0.3)
    speed = MIN_SPEED + scale * (BASE_SPEED - MIN_SPEED)

    if center and not left and not right:
        return speed, speed
    elif left and not center:
        return 0.5 * speed, speed
    elif right and not center:
        return speed, 0.5 * speed
    elif center and left and not right:
        return 0.3 * speed, speed
    elif center and right and not left:
        return speed, 0.3 * speed
    else:
        return 0.5 * speed, 0.5 * speed

# ---------- Main Loop ----------
while robot.step(timestep) != -1:
    current_pos = gps.getValues()[0:2]
    heading = get_heading()

    draw_map(path, current_pos)

    if current_target_index >= len(path):
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        print("Destination reached!")
        break

    target_node = path[current_target_index]
    target_pos = node_coords[target_node]
    dist = distance_to_target(current_pos, target_pos)
    angle_diff = angle_to_target(current_pos, heading, target_pos)

    if dist < DISTANCE_TOLERANCE:
        print(f"Reached {target_node}")
        current_target_index += 1
        state = "TURNING"
        continue

    if state == "TURNING":
        if abs(angle_diff) > ANGLE_TOLERANCE:
            direction = 1 if angle_diff > 0 else -1
            leftSpeed = -ROTATION_SPEED * direction
            rightSpeed = ROTATION_SPEED * direction
        else:
            print(f"Turned toward {target_node}")
            state = "FOLLOWING"
            leftSpeed = 0
            rightSpeed = 0
    elif state == "FOLLOWING":
        leftSpeed, rightSpeed = line_following(dist)

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

# Final plot after simulation
plt.ioff()
draw_map(path, current_pos)
plt.show()
