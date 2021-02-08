import enum
from .utils import *

class Color(enum.Enum):
    RED = (255, 0, 0)
    BLUE = (0, 0, 255)
    GREEN = (0, 90, 0)
    UNKNOWN = (88, 92, 99)

    @staticmethod
    def get_color(rgb):
        rgb = normalize(np.array(rgb))
        return min(list(Color), key=lambda x: np.linalg.norm(rgb - normalize(np.array(x.value))))


class Field:
    thickness = 0.01
    walls = [np.array([-1.2 + thickness, -1.2 + thickness]), np.array([-1.2 + thickness, 1.2 - thickness]), np.array([1.2 - thickness, 1.2 - thickness]), np.array([1.2 - thickness, -1.2 + thickness])]

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
        offset += Field.thickness
        return -1.2 + offset < pos[0] < 1.2 - offset and -1.2 + offset < pos[1] < 1.2 - offset

    def __init__(self):
        self.field = {}
        self.additions = {}
        self.color_changes = {}
        self.counter = 0
        self.unvisited = set()
        self.search_spots = [np.array([0, -1]), np.array([0, 1])]

    def add_block(self, pos, color=Color.UNKNOWN, use_field=True):
        if use_field:
            self.field[self.counter] = [pos, color]
            self.unvisited.add(self.counter)
        self.additions[self.counter] = [pos, color]
        self.counter += 1

    def get_block_pos(self, blockID):
        return self.field[blockID][0]

    def get_block_color(self, blockID):
        return self.field[blockID][1]

    def set_block_color(self, blockID, color: Color):
        self.field[blockID][1] = color
        self.color_changes[blockID] = color

    def remove_block(self, blockID):
        del self.field[blockID]

    def get_additions(self, use_id=True):
        s = ''
        for i in self.additions:
            if use_id:
                s += str(i) + ' '
            else:
                s += '-1 '
            s += ' '.join(map(str, self.additions[i][0]))
            s += ' ' + self.additions[i][1].name + ' '
        self.additions.clear()
        return s

    def get_color_changes(self):
        s = ''
        for i in self.color_changes:
            s += str(i) + ' ' + self.color_changes[i].name + ' '
        self.color_changes.clear()
        return s


    def parse(self, radio_input:str, use_id=True, mark_changes=False, threshold=None):
        radio_input = radio_input.split(' ')
        for i in range(0, len(radio_input), 4):
            if radio_input[i] == '':
                continue
            if use_id:
                id = int(radio_input[i])
            else:
                id = self.counter
            pos = np.array([float(radio_input[i+1]), float(radio_input[i+2])])
            if threshold is None or not self.contains_point(pos, threshold):
                self.field[id] = [pos, Color[radio_input[i+3]]]
                self.unvisited.add(id)
                if mark_changes:
                    self.additions[id] = self.field[id]
                if not use_id:
                    self.counter += 1

    def parse_color_changes(self, radio_input:str):
        radio_input = radio_input.split(' ')
        for i in range(0, len(radio_input), 2):
            if radio_input[i] == '':
                continue
            self.field[int(radio_input[i])][1] = Color[radio_input[i + 1]]

    def contains_point(self, pos, threshold=0.05):
        for block in self.field:
            if np.linalg.norm(self.field[block][0] - pos) <= threshold:
                return True
        for block in self.additions:
            if np.linalg.norm(self.additions[block][0] - pos) <= threshold:
                return True
        return False

    def allocate_block(self, robotData, otherRobotData=None):
        id = -1
        min_dist = 1000
        for key in self.field.keys():
            if otherRobotData is not None and key == otherRobotData.targetBlock:
                continue
            if self.field[key][1] == robotData.color or self.field[key][1] == Color.UNKNOWN:
                dist = np.linalg.norm(self.field[key][0] - robotData.position)
                if dist < min_dist:
                    min_dist = dist
                    id = key
        return id

    def allocate_search(self, pos):
        if len(self.search_spots) <= 0:
            return None
        pos = min(self.search_spots, key=lambda x: np.linalg.norm(x - pos))
        self.search_spots.remove(pos)
        return pos

