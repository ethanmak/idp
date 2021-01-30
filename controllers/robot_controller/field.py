import enum

class Color(enum.Enum):
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    UNKNOWN = (88, 92, 99)

class Field:
    walls = [(-1.2, -1.2), (1.2, 1.2)]

    @staticmethod
    def distance_to_wall(self, position, rotation):
        return 0

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

