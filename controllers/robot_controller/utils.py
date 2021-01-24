import math
import enum


class LogicState(enum.Enum):
    SEARCH = enum.auto()
    TRAVEL = enum.auto()
    CAPTURE = enum.auto()
    DEPOSIT = enum.auto()


class MovementCommand(enum.Enum):
    TURN = enum.auto()
    POINT = enum.auto()
    FORWARD = enum.auto()
    SWEEP = enum.auto()
    OPEN = enum.auto()
    CLOSE = enum.auto()


def clamp(x, low, high):
    return max(min(x, high), low)