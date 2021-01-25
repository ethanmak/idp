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
target = None

depositBox = None

def setup():
    global robot, depositBox
    robot = Robot(controller.Robot())
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    depositBox = np.array(robot.get_gps_position()[:2])

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
        movementQueue.put((RobotCommand.OPEN,))
        movementQueue.put((RobotCommand.TRAVEL, (1, 1)))
        movementQueue.put((RobotCommand.CLOSE,))
    elif currentLogicState == LogicState.SEARCH:
        movementQueue.put((RobotCommand.SWEEP,))
    elif currentLogicState == LogicState.TRAVEL:
        movementQueue.put((RobotCommand.TURN, 100))
        movementQueue.put((RobotCommand.POINT, (1,1)))
    elif currentLogicState == LogicState.DEPOSIT:
        movementQueue.put((RobotCommand.OPEN,))
        movementQueue.put((RobotCommand.FORWARD, - 0.1))
        movementQueue.put((RobotCommand.CLOSE,))
        movementQueue.put((RobotCommand.FORWARD, 0.1))
    elif currentLogicState == LogicState.TRAVEL_BACK:
        movementQueue.put((RobotCommand.TURN, 100))
        movementQueue.put((RobotCommand.POINT, depositBox))
    currentLogicState = None

def movement_state_machine():
    global currentMovementState, target
    if currentMovementState is None:
        command = movementQueue.get()
        currentMovementState = command[0]
        target = command[1]
        if currentMovementState == RobotCommand.FORWARD: # these sets of if statements are use for movement commands that must be executed only once
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
    logicQueue.put((LogicState.TRAVEL, np.array()))
    while robot.step():
        process_radio_signals()
        update()
        logic_state_machine()
        movement_state_machine()
        broadcast_update()
