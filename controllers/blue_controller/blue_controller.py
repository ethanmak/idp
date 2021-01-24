# blue controller (master controller)
import controller
import queue
from robot_controller import *

robot = None # type: Robot
blueRobotData = RobotData()
redRobotData = RobotData()
sendChannel, receiveChannel = 0, 1

logicQueue = queue.Queue()
currentLogicState = None
movementQueue = queue.Queue()
currentMovementState = None


def setup():
    global robot
    robot = Robot(controller.Robot())
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)

def exit():
    pass

def update():
    blueRobotData.position = robot.get_gps_position()

def process_radio_signals():
    while robot.radio.hasNext():
        data = robot.radio.next()
        if data == '':
            continue
        if data.find('UPDATE:') != -1:
            redRobotData.parse(data)

def broadcast_update():
    robot.radio.send('UPDATE:' + repr(blueRobotData))

def logic_state_machine():
    global currentLogicState
    if currentLogicState is None and movementQueue.empty():
        currentLogicState = logicQueue.get()
    if currentLogicState == LogicState.CAPTURE:
        movementQueue.put((MovementCommand.OPEN,))
        movementQueue.put((MovementCommand.TRAVEL,(1,1)))
    elif currentLogicState == LogicState.SEARCH:
        movementQueue.put((MovementCommand.SWEEP,))
    elif currentLogicState == LogicState.TRAVEL:
        pass
    elif currentLogicState == LogicState.DEPOSIT:
        pass
    currentLogicState = None

def process_movement_command():
    global currentMovementState
    command = None
    if currentMovementState is None:
        command = movementQueue.get()
        currentMovementState = command[0]
    if currentMovementState == MovementCommand.TURN:
        robot.turn_degrees(command[1])
    elif currentMovementState == MovementCommand.POINT:
        pass
    elif currentMovementState == MovementCommand.OPEN:
        pass
    elif currentMovementState == MovementCommand.CLOSE:
        pass
    elif currentMovementState == MovementCommand.SWEEP:
        pass

def movement_state_machine():
    global currentMovementState
    if currentMovementState == MovementCommand.TURN:
        if not robot.is_moving():
            currentMovementState = None
    elif currentMovementState == MovementCommand.POINT:
        if False: #change to be tolerance within GPS Coordinates
            currentMovementState = None
    elif currentMovementState == MovementCommand.OPEN or currentMovementState == MovementCommand.CLOSE:
        if not robot.is_gate_moving():
            currentMovementState = None
    elif currentMovementState == MovementCommand.SWEEP:
        pass

if __name__ == '__main__':
    setup()
    logicQueue.put(LogicState.SEARCH)
    while robot.step():
        process_radio_signals()
        update()
        logic_state_machine()
        movement_state_machine()
        broadcast_update()
    exit()
