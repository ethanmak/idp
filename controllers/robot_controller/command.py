import enum


class LogicCommand(enum.Enum):
    """
    This enum holds all the commands associated with the logic of the robot, as explained below:

    SEARCH: Sweep in place for blocks in the field

    TRAVEL: Turn and move to a certain point, typically a block location
    Parameter: Target point to move robot to

    CAPTURE: Intake the block in front of the robot

    DEPOSIT: Deposit the block within the cavity, assuming already in the deposit box

    TRAVEL_BACK: Travel to the deposit box

    COLOR: Determine the color of the block in front of the robot, and queue LogicCommands according to color

    DELAY: Delay the execution of commands by a few seconds
    Parameter: Seconds to delay by
    """

    SEARCH = enum.auto()
    TRAVEL = enum.auto()
    CAPTURE = enum.auto()
    DEPOSIT = enum.auto()
    TRAVEL_BACK = enum.auto()
    COLOR = enum.auto()
    DELAY = enum.auto()


class RobotCommand(enum.Enum):
    """
    This enum holds all commands associated with the direct interfacing of the robot, excluding color detection

    TURN: Turn to a specific heading
    Parameter: heading to turn to

    POINT: Travel to a specific point
    Parameter: Target point

    FORWARD: Move forward for specified number of seconds
    Parameter: Seconds to move forward

    BACKWARD: Move backward for specified number of seconds
    Parameter: Seconds to move backward

    SWEEP: Search in place for new blocks within range of distance sensor
    Parameter: Whether to log new blocks in own field representation (used to maintain master-follower protocol)

    OPEN: Opens gate

    CLOSE: Closes gate

    DELAY: Pause command execution by specified number of seconds
    Parameter: Number of seconds to delay by

    PAUSE: Pause command execution indefinitely

    REMOVE_BLOCK: Remove block from field representation
    Parameter: Block ID to remove
    """

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
