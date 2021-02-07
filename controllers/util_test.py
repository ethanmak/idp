from robot_controller.utils import *
from robot_controller.field import *
import numpy as np

if __name__ == '__main__':
    x = np.array([1, -1])
    y = np.array([-1, 1])
    print(vector_degree(y - x))
    print(Field.distance_to_wall(x, 90))
    a = np.array([0.5, 0])
    print(list(Color))
