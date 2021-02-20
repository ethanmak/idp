import enum

from . import RobotData
from .utils import *

class Color(enum.Enum):
    """
    This enum holds the color definitions that can appear on the field, and can determine them based on rgb values
    """
    RED = (255, 0, 0)
    BLUE = (0, 0, 255)
    GREEN = (0, 90, 0)
    UNKNOWN = (88, 92, 99)

    @staticmethod
    def get_color(rgb: tuple):
        """
        Finds the closest Color based on RGB values

        :param rgb: Tuple of RGB values to be analyzed
        :return: Closest Color enum value to RGB
        """
        rgb = normalize(np.array(rgb))
        return min(list(Color), key=lambda x: np.linalg.norm(rgb - normalize(np.array(x.value))))


class Field:
    """
    This class holds the field representation as well as functions to turn it to strings and manipulate the field
    """
    thickness = 0.01
    walls = [np.array([-1.2 + thickness, -1.2 + thickness]), np.array([-1.2 + thickness, 1.2 - thickness]), np.array([1.2 - thickness, 1.2 - thickness]), np.array([1.2 - thickness, -1.2 + thickness])]

    @staticmethod
    def distance_to_wall(origin: np.ndarray, rotation: float) -> float:
        """
        Determines distance of a robot pose to wall by raytracing


        :param origin:
        :param rotation: Rotation of pose to be analyzed
        :return: Distance of pose ray trace to wall, -2 if no intersection
        """
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
    def point_wall_distance(pos: np.ndarray) -> float:
        """
        Returns the minimum distance of a point to the walls

        :param pos: Coordinates of point
        :return: Distance to the wall
        """
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
    def in_bounds(pos: np.ndarray, offset: float = 0):
        """
        Returns whether a point is in bounds or not

        :param pos: Coordinates of point
        :param offset: Offset to make bounds smaller or larger; +ve makes bounds smaller, -ve makes bounds larger
        :return: True if point is in bounds, False otherwise
        """
        offset += Field.thickness
        return -1.2 + offset < pos[0] < 1.2 - offset and -1.2 + offset < pos[1] < 1.2 - offset

    @staticmethod
    def in_deposit_boxes(pos, offset=0):
        """
        Returns if point is inside field deposit boxes

        :param pos: Coordinates of point
        :param offset: Offset to make boxes smaller or larger; +ve makes boxes smaller, -ve makes boxes larger
        :return: True if point is inside deposit boxes, False otherwise
        """
        return 0.8 + offset <= pos[0] <= 1.2 - offset and not -0.8 - offset < pos[1] < 0.8 + offset

    def __init__(self):
        """
        Initializer
        """
        self.field = {}
        self.additions = {}
        self.color_changes = {}
        self.counter = 0
        self.unvisited = set()
        self.search_spots = [np.array([0, -1]), np.array([0, 1]), np.array([0, 0])]
        self.deletions = []

    def add_block(self, pos: np.ndarray, color: Color = Color.UNKNOWN, use_field: bool = True) -> None:
        """

        :param pos:
        :param color:
        :param use_field:
        :return:
        """
        if use_field:
            self.field[self.counter] = [pos, color]
            self.unvisited.add(self.counter)
        self.additions[self.counter] = [pos, color]
        self.counter += 1

    def get_block_pos(self, blockID: int) -> np.ndarray:
        return self.field[blockID][0]

    def get_block_color(self, blockID: int) -> Color:
        return self.field[blockID][1]

    def set_block_color(self, blockID: int, color: Color) -> None:
        self.field[blockID][1] = color
        self.color_changes[blockID] = color

    def remove_block(self, blockID: int) -> None:
        del self.field[blockID]
        self.deletions.append(blockID)

    def get_additions(self, use_id: bool = True) -> str:
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

    def get_color_changes(self) -> str:
        s = ''
        for i in self.color_changes:
            s += str(i) + ' ' + self.color_changes[i].name + ' '
        self.color_changes.clear()
        return s

    def get_deletions(self) -> str:
        res = ' '.join(map(str, self.deletions))
        self.deletions.clear()
        return res

    def parse(self, radio_input: str, use_id: bool = True, mark_changes: bool = False, threshold: float = None) -> None:
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

    def parse_color_changes(self, radio_input: str) -> None:
        """


        :param radio_input:
        :return:
        """
        radio_input = radio_input.split(' ')
        for i in range(0, len(radio_input), 2):
            if radio_input[i] == '':
                continue
            self.field[int(radio_input[i])][1] = Color[radio_input[i + 1]]

    def parse_deletions(self, radio_input: str) -> list[int]:
        """


        :param radio_input:
        :return:
        """
        radio_input = list(map(int, radio_input.split(' ')))
        for i in radio_input:
            del self.field[i]
        return radio_input

    def contains_point(self, pos: np.ndarray, threshold: float = 0.05) -> bool:
        """


        :param pos:
        :param threshold:
        :return:
        """
        for block in self.field:
            if np.linalg.norm(self.field[block][0] - pos) <= threshold:
                return True
        for block in self.additions:
            if np.linalg.norm(self.additions[block][0] - pos) <= threshold:
                return True
        return False

    def allocate_block(self, robotData: RobotData, otherRobotData: RobotData) -> int:
        """


        :param robotData:
        :param otherRobotData:
        :return:
        """
        id = -1
        min_dist = 1000
        intersect = False
        cont_main_loop = False
        for key in self.field.keys():
            if key == otherRobotData.targetBlock:
                continue
            for key2 in self.field.keys():
                if key == key2:
                    continue
                if distance_segment_point_no_end(robotData.position, self.field[key][0], self.field[key2][0]) < 0.09 + 0.025:
                    cont_main_loop = True
                    intersect = True
                    break
            if cont_main_loop:
                cont_main_loop = False
                continue
            if self.field[key][1] == robotData.color or self.field[key][1] == Color.UNKNOWN:
                dist = np.linalg.norm(self.field[key][0] - robotData.position)
                if dist < min_dist:
                    min_dist = dist
                    id = key
        if id == -1 and intersect:
            return -2
        return id

    def allocate_search(self, robotData: RobotData) -> np.ndarray:
        """


        :param robotData:
        :return:
        """
        if len(self.search_spots) <= 0:
            return None
        id = min(range(0, len(self.search_spots)), key=lambda x: np.linalg.norm(self.search_spots[x] - robotData.position))
        pos = self.search_spots[id]
        del self.search_spots[id]
        return pos

