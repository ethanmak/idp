import enum
import numpy as np


class LogicState(enum.Enum):
    SEARCH = enum.auto()
    TRAVEL = enum.auto()
    CAPTURE = enum.auto()
    DEPOSIT = enum.auto()


class RobotCommand(enum.Enum):
    TURN = enum.auto()
    POINT = enum.auto()
    FORWARD = enum.auto()
    SWEEP = enum.auto()
    OPEN = enum.auto()
    CLOSE = enum.auto()
    COLOR = enum.auto()

def normalize(x):
    return x / np.linalg.norm(x)

def clamp(x, low, high):
    return max(min(x, high), low)