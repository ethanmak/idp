from robot_controller.utils import *
from robot_controller.field import *
import numpy as np

if __name__ == '__main__':
    x = np.array([1, -1])
    y = np.array([-1, -1])
    a = np.array([-2, 0])
    b = np.array([-2, 1])
    print(line_segments_intersect(x, y, a, b))
