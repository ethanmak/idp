import enum
from typing import List

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
    # thickness of the wall
    thickness = 0.01
    # a list of points of the four corners of the walls
    walls = [np.array([-1.2 + thickness, -1.2 + thickness]), np.array([-1.2 + thickness, 1.2 - thickness]), np.array([1.2 - thickness, 1.2 - thickness]), np.array([1.2 - thickness, -1.2 + thickness])]

    @staticmethod
    def distance_to_wall(origin: np.ndarray, rotation: float) -> float:
        """
        Determines distance of a pose to wall by raytracing

        This function draws a ray from the origin in the direction of its rotation. Then using the intersection between rays and line segments, it determines the distance

        :param origin: Position of the pose to be analyzed
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
        self.field = {}  # field representation of blocks
        self.additions = {}  # log of additions for get_additions
        self.color_changes = {}  # log of color changes for get_color_changes
        self.counter = 0  # used for auto assigning IDs
        self.unvisited = set()  # set of unvisited blocks
        self.search_spots = [np.array([0, -1]), np.array([0, 1]), np.array([0, 0])]  # predefined search spots to cover field
        self.deletions = []  # log of deletions for get_deletions

    def add_block(self, pos: np.ndarray, color: Color = Color.UNKNOWN, use_field: bool = True) -> None:
        """
        Adds block to field representation and logs for next call to get_additions

        :param pos: Position of block
        :param color: Color of block (Default is Color.UNKNOWN)
        :param use_field: Whether to add this to the field representation (Default is True)
        :return: None
        """
        if use_field:
            self.field[self.counter] = [pos, color]
            self.unvisited.add(self.counter)
        self.additions[self.counter] = [pos, color]
        self.counter += 1

    def get_block_pos(self, blockID: int) -> np.ndarray:
        """
        Returns position of block

        :param blockID: Block ID of block to look up
        :return: Position of block
        """
        return self.field[blockID][0]

    def get_block_color(self, blockID: int) -> Color:
        """
        Returns color of block

        :param blockID: Block ID of block to look up
        :return: Color of block
        """
        return self.field[blockID][1]

    def set_block_color(self, blockID: int, color: Color) -> None:
        """
        Sets the block color and logs for next call to get_color_changes

        :param blockID: Block ID of block to change color
        :param color: Color of block
        :return: None
        """
        self.field[blockID][1] = color
        self.color_changes[blockID] = color

    def remove_block(self, blockID: int) -> None:
        """
        Removes block from field representation and logs for next call to get_deletions

        :param blockID: Block ID to remove
        :return: None
        """
        del self.field[blockID]
        self.deletions.append(blockID)

    def get_additions(self, use_id: bool = True) -> str:
        """
        Returns additions from last call to get_additions to this call in a parsable string by parse_additions

        :param use_id: Whether to take into account the block IDs assigned automatically, or assign dummy IDs (True by default)
        :return: Parsable string with additions from last call to get_additions to this call
        """
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
        """
        Returns a string that contains color changes of blocks from last call to get_color_changes that can be parsed by parse_color_changes

        :return: Parsable string with block IDs changed and colors associated
        """
        s = ''
        for i in self.color_changes:
            s += str(i) + ' ' + self.color_changes[i].name + ' '
        self.color_changes.clear()
        return s

    def get_deletions(self) -> str:
        """
        Returns a string that contains block deletions from last call to get_deletions that can be parsed by parse_deletions

        :return: Parsable string
        """
        res = ' '.join(map(str, self.deletions))
        self.deletions.clear()
        return res

    def parse_additions(self, radio_input: str, use_id: bool = True, mark_changes: bool = False, threshold: float = None) -> None:
        """
        Adds blocks to field representation as given by string returned by get_additions

        :param radio_input: Parsable string given by get_additions
        :param use_id: Whether to use block id supplied by string or generate new ones (used for master-follower protocol)
        :param mark_changes: Whether to mark additions as new additions to be sent out in next call to get_additions
        :param threshold: Distance to which block is considered covered by current field representation
        :return: None
        """
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
        Changes colors of blocks as given by parsable string returned by get_color_changes

        :param radio_input: Parsable string from get_color_changes
        :return: None
        """
        radio_input = radio_input.split(' ')
        for i in range(0, len(radio_input), 2):
            if radio_input[i] == '':
                continue
            self.field[int(radio_input[i])][1] = Color[radio_input[i + 1]]

    def parse_deletions(self, radio_input: str) -> List[int]:
        """
        Deletes blocks as given by parsable string returned by get_deletions

        :param radio_input: Parsable string
        :return: List of deleted block IDs
        """
        radio_input = list(map(int, radio_input.split(' ')))
        for i in radio_input:
            del self.field[i]
        return radio_input

    def contains_point(self, pos: np.ndarray, threshold: float = 0.05) -> bool:
        """
        Returns if a point is already covered by a block in the field representation

        :param pos: Coordinates of point
        :param threshold: Distance under which point is considered covered
        :return: True if covered, False otherwise
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
        Allocates a block ID based on a set of criteria for the target robot
        Criteria are as below:
        Line of sight (the robot cannot travel to blocks where other blocks are blocking the path)
        Distance to non-target robot (the robot should not travel to places where it may collide with the other robot)
        Distance to target robot

        :param robotData: RobotData of target robot
        :param otherRobotData: RobotData of other (non-target) robot
        :return: Block ID allocated for robot
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
        Allocates a position for the robot to search to, based on the current position of the robot, and removes it from list

        :param robotData: RobotData of target robot
        :return: The point to search from, None if there are none left
        """
        if len(self.search_spots) <= 0:
            return None
        id = min(range(0, len(self.search_spots)), key=lambda x: np.linalg.norm(self.search_spots[x] - robotData.position))
        pos = self.search_spots[id]
        del self.search_spots[id]
        return pos

