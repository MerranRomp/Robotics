


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