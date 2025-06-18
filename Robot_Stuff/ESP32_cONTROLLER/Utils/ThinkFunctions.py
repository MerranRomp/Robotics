import heapq
from nodes import graph
from math import sqrt, atan2, pi

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

def dijkstra(graph, start, goal):
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

def pid_update(error, Kp, Ki, Kd):
    global pid_error_sum, last_pid_error
    pid_error_sum += error
    d_error = error - last_pid_error
    last_pid_error = error
    return Kp * error + Ki * pid_error_sum + Kd * d_error


## -- TURN DETECTION -- ##
def get_turn_direction(prev_node, current_node, next_node):
    def angle(a, b):
        ax, ay = graph[a]['pos']
        bx, by = graph[b]['pos']
        return atan2(by - ay, bx - ax)
   
    a1 = angle(prev_node, current_node)
    a2 = angle(current_node, next_node)
    delta = (a2 - a1 + pi) % (2 * pi) - pi

    if delta > 0.2:
        return 'turn_left'
    elif delta < -0.2:
        return 'turn_right'
    else:
        return 'line_following'