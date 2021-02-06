import numpy as np

def normalize(x):
    return x / np.linalg.norm(x)

def clamp(x, low, high):
    return max(min(x, high), low)

def vector_degree(x, flip_x=False):
    index = 1 if len(x) == 2 else 2
    flip_x = -1 if flip_x else 1
    deg = np.degrees(np.arctan2(x[index], x[0] * flip_x))
    return deg if deg > 0 else deg + 360

def min_abs(*x):
    return min(x, key=abs)

def angle_subtract(x1, x2):
    return min_abs(x1 - x2, x1 - x2 + 360, x1 - x2 - 360)

def degree_to_vector(degree):
    degree = np.radians(degree)
    return np.array([np.cos(degree), np.sin(degree)])

def add_distance_vector(vector, distance):
    return vector + distance * normalize(vector)
