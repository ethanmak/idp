import enum


class LogicCommand(enum.Enum):
    SEARCH = enum.auto()
    TRAVEL = enum.auto()
    CAPTURE = enum.auto()
    DEPOSIT = enum.auto()
    TRAVEL_BACK = enum.auto()
    COLOR = enum.auto()
    DELAY = enum.auto()


class RobotCommand(enum.Enum):
    TURN = enum.auto()
    POINT = enum.auto()
    FORWARD = enum.auto()
    BACKWARD = enum.auto()
    SWEEP = enum.auto()
    OPEN = enum.auto()
    CLOSE = enum.auto()
    DELAY = enum.auto()
    PAUSE = enum.auto()
    REMOVE_BLOCK = enum.auto()
