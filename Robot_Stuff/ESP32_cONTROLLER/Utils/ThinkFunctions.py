import heapq
from Utils.nodes import graph
from math import atan2, pi, cos, sin

 ##----------------IR GROUND SENSOR----------------##

 # line_following/think.py

weights = [-2, -1, 0, 1, 2]

def line_position_binary(binary_vals):
    return sum(w * b for w, b in zip(weights, binary_vals)) / max(sum(binary_vals), 1)

def line_position_inverse(inverse_vals):
    return sum(w * v for w, v in zip(weights, inverse_vals)) / max(sum(inverse_vals), 1)

def compute_error(norm_or_bin_vals, method='inverse'):
    if method == 'inverse':
        return line_position_inverse(norm_or_bin_vals)
    elif method == 'binary':
        return line_position_binary(norm_or_bin_vals)
    else:
        raise ValueError("Unknown method. Use 'inverse' or 'binary'")
    


# ##----------------PATHFINDING----------------##

def dijkstra(start, goal):
    from nodes import graph  # uses the shared graph
    queue = [(0, start, [])]
    visited = set()

    while queue:
        cost, node, path = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        path = path + [node]

        if node == goal:
            return path, cost

        for neighbor, weight in graph[node]['neighbors'].items():
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path))

    return None, float('inf')

## ##----------------TURN DETECTION----------------##
def get_angle(p1, p2):
    return atan2(p2[1] - p1[1], p2[0] - p1[0])

def get_turn_directions(graph, path):
    directions = []
    for i in range(1, len(path)-1):
        p0 = graph[path[i-1]]['pos']
        p1 = graph[path[i]]['pos']
        p2 = graph[path[i+1]]['pos']

        angle1 = get_angle(p0, p1)
        angle2 = get_angle(p1, p2)
        delta = angle2 - angle1
        delta = (delta + pi) % (2 * pi) - pi  # Normalize to [-π, π]

        if delta > 0.2:
            turn = 'left'
        elif delta < -0.2:
            turn = 'right'
        else:
            turn = 'straight'

        directions.append((path[i], turn))
    return directions


## -- PID CONTROL -- ##
pid_error_sum = 0
last_pid_error = 0

def pid_update(error, Kp, Ki, Kd, update=True):
    global pid_error_sum, last_pid_error, last_pid_output

    if update:
        pid_error_sum += error
        d_error = error - last_pid_error
        last_pid_error = error
        last_pid_output = Kp * error + Ki * pid_error_sum + Kd * d_error
    # If not updating (line lost), return last correction
    return last_pid_output


##----------------ROBOT POSE UPDATE----------------##
def normalize_angle_rad(angle_rad):
    """Normalize angle to [-π, π) radians."""
    from math import pi
    angle_rad = (angle_rad + pi) % (2 * pi) - pi
    return angle_rad

def update_pose(x, y, theta, delta_left_cm, delta_right_cm, wheel_base_cm):
    """
    Updates the robot's pose based on encoder distances.

    Inputs:
        x, y           : current position (cm)
        theta          : current heading (radians)
        delta_left_cm  : left wheel distance (cm)
        delta_right_cm : right wheel distance (cm)
        wheel_base_cm  : distance between wheels (cm)

    Returns:
        Updated x, y, theta (with theta ∈ [-π, π))
    """
    delta_d = (delta_right_cm + delta_left_cm) / 2.0
    delta_theta = (delta_right_cm - delta_left_cm) / wheel_base_cm

    theta += delta_theta
    theta = normalize_angle_rad(theta)

    x += delta_d * cos(theta)
    y += delta_d * sin(theta)

    return x, y, theta

#----------------WHEEL SPEED CALCULATION----------------##
def get_wheel_speeds(ticks_left, ticks_right, dt_ms, ppr, gear_ratio, wheel_diameter_cm):
    """
    Calculate left and right wheel speeds in cm/s.

    Inputs:
        ticks_left / ticks_right: encoder ticks since last update
        dt_ms: time interval in milliseconds
        ppr: pulses per revolution
        gear_ratio: gear reduction ratio
        wheel_diameter_cm: wheel diameter in cm

    Returns:
        (speed_left_cm_s, speed_right_cm_s)
    """
    if dt_ms == 0:
        return 0.0, 0.0  # Prevent division by zero

    cm_per_tick = (pi * wheel_diameter_cm) / (ppr * gear_ratio)
    delta_t_s = dt_ms / 1000.0

    speed_left = ticks_left * cm_per_tick / delta_t_s
    speed_right = ticks_right * cm_per_tick / delta_t_s

    return speed_left, speed_right



def angle_difference(current, start):
    delta = current - start
    while delta > pi:
        delta -= 2 * pi
    while delta < -pi:
        delta += 2 * pi
    return abs(delta)
