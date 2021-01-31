import enum
from .utils import *

class Color(enum.Enum):
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    UNKNOWN = (88, 92, 99)

class Field:
    walls = [np.array([-1.2, -1.2]), np.array([-1.2, 1.2]), np.array([1.2, 1.2]), np.array([1.2, -1.2])]

    @staticmethod
    def distance_to_wall(origin, rotation):
        direction = degree_to_vector(rotation)
        v3 = np.array([-direction[1], direction[0]])
        for i in range(-1, len(Field.walls) - 1):
            v1 = origin - Field.walls[i]
            v2 = Field.walls[i+1] - Field.walls[i]
            dot = np.dot(v2, v3)
            if np.isclose(dot, 0):
                continue
            t1 = np.cross(v2, v1) / dot
            t2 = np.dot(v1, v3) / dot
            if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
                return np.linalg.norm(origin + t1 * direction - origin)
        return -2

    def __init__(self):
        self.field = {}
        self.changes = {}
        self.counter = 0

    def add_block(self, pos, color=Color.UNKNOWN):
        self.field[self.counter] = [pos, color]
        self.changes[self.counter] = [pos, color]
        self.counter += 1

    def remove_block(self, blockID):
        del self.field[blockID]

    def get_changes(self):
        return ''

