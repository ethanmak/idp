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

    @staticmethod
    def point_wall_distance(pos):
        dists = []
        for i in range(-1, len(Field.walls) - 1):
            a = Field.walls[i]
            b = Field.walls[i+1]
            if a[0] == b[0]:
                dists.append(abs(a[0] - pos[0]))
            else:
                dists.append(abs(a[1] - pos[1]))
        return min(dists)

    @staticmethod
    def in_bounds(pos, offset=0):
        offset += 0.01
        return -1.2 + offset < pos[0] < 1.2 - offset and -1.2 + offset < pos[1] < 1.2 - offset

    def __init__(self):
        self.field = {}
        self.changes = {}
        self.counter = 0

    def add_block(self, pos, color=Color.UNKNOWN, use_field=True):
        if use_field:
            self.field[self.counter] = [pos, color]
        self.changes[self.counter] = [pos, color]
        self.counter += 1

    def remove_block(self, blockID):
        del self.field[blockID]

    def get_additions(self, use_id=True):
        s = ''
        for i in self.changes:
            if use_id:
                s += str(i) + ' '
            else:
                s += '-1 '
            s += ' '.join(map(str, self.changes[i][0]))
            s += ' ' + self.changes[i][1].name + ' '
        self.changes.clear()
        return s

    def parse(self, radio_input:str, use_id=True):
        radio_input = radio_input[6:].split(' ')
        for i in range(0, len(radio_input), 4):
            if use_id:
                id = int(radio_input[i])
            else:
                id = self.counter
            self.field[id] = [np.array([float(radio_input[i+1]), float(radio_input[i+2])]), Color[radio_input[i+3]]]
            if not use_id:
                self.counter += 1

    def contains_point(self, pos, threshold=0.05):
        for block in self.field:
            if np.linalg.norm(self.field[block][0] - pos) <= threshold:
                return True

