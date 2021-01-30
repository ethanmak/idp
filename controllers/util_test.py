from robot_controller.utils import *
import numpy as np

if __name__ == '__main__':
    x = np.array([1, -1])
    y = np.array([-1, 1])
    print(vector_degree(y - x))