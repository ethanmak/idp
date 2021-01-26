import enum
import numpy as np

def normalize(x):
    return x / np.linalg.norm(x)

def clamp(x, low, high):
    return max(min(x, high), low)

def vector_degree(x):
    return np.degrees(np.arctan2(x[1], x[0]))