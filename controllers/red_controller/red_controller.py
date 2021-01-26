#red controller (slave controller)
import controller
from robot_controller import *
import queue

robot = None  # type: Robot
blueRobotData = RobotData()
redRobotData = RobotData()
sendChannel, receiveChannel = 1, 0

logicQueue = queue.Queue()
currentLogicState = None
movementQueue = queue.Queue()
currentMovementState = None

def setup():
    global robot
    robot = Robot(controller.Robot())
    robot._init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)

def update():
    redRobotData.position = robot.get_gps_position()

def process_radio_signals():
    global currentLogicState, currentMovementState
    while robot.radio.hasNext():
        data = robot.radio.next()
        if data == '':
            continue
        if data.find('UPDATE:') != -1:
            blueRobotData.parse(data)
        elif data.find('END') != -1:
            robot.stop_motors()
            while not movementQueue.empty():
                movementQueue.get()
            while not logicQueue.empty():
                logicQueue.get()
            currentLogicState = None
            currentMovementState = None


def broadcast_update():
    robot.radio.send('UPDATE:' + repr(redRobotData))

def logic_state_machine():
    global currentLogicState
    if currentLogicState is None and movementQueue.empty():
        currentLogicState = logicQueue.get()
    if currentLogicState == LogicCommand.CAPTURE:
        movementQueue.put((RobotCommand.OPEN,))
        movementQueue.put((RobotCommand.TRAVEL, (1, 1)))
        movementQueue.put((RobotCommand.CLOSE,))
    elif currentLogicState == LogicCommand.SEARCH:
        movementQueue.put((RobotCommand.SWEEP,))
    elif currentLogicState == LogicCommand.TRAVEL:
        pass
    elif currentLogicState == LogicCommand.DEPOSIT:
        movementQueue.put((RobotCommand.OPEN,))
        movementQueue.put()
    elif currentLogicState == LogicCommand.TRAVEL_BACK:
        pass
    currentLogicState = None

def movement_state_machine():
    global currentMovementState, target
    if currentMovementState is None:
        command = movementQueue.get()
        currentMovementState = command[0]
        target = command[1]
        if currentMovementState == RobotCommand.FORWARD:  # these sets of if statements are use for movement commands that must be executed only once
            robot.go_forward_distance(target)
        elif currentMovementState == RobotCommand.SWEEP:
            robot.turn_degrees(360, 0.1)
    if currentMovementState == RobotCommand.TURN:
        robot.turn_degrees(target)
        if not robot.is_moving():
            currentMovementState = None
    elif currentMovementState == RobotCommand.FORWARD:
        if not robot.is_moving():
            currentMovementState = None
    elif currentMovementState == RobotCommand.POINT:
        if robot.move_to_target(target):
            currentMovementState = None
    elif currentMovementState == RobotCommand.OPEN:
        if not robot.is_gate_moving():
            currentMovementState = None
    elif currentMovementState == RobotCommand.SWEEP:
        if not robot.is_moving():
            currentMovementState = None


if __name__ == '__main__':
    setup()
    while robot.step():
        process_radio_signals()
        update()
        logic_state_machine()
        movement_state_machine()
        broadcast_update()
