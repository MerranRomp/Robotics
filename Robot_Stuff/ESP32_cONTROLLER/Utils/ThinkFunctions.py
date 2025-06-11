


 ##----------------IR GROUND SENSOR----------------##

 # line_following/think.py

weights = [-2, -1, 0, 1, 2]

def line_position_binary(binary_vals):
    return sum(w * b for w, b in zip(weights, binary_vals)) / max(sum(binary_vals), 1)

def line_position_inverse(inverse_vals):
    return sum(w * v for w, v in zip(weights, inverse_vals)) / max(sum(inverse_vals), 1)

