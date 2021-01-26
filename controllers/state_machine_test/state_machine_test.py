# blue controller (master controller)
import controller
import queue
from robot_controller import *
from robot_controller.logic import LogicStateMachine

robot = None  # type: Robot
blueRobotData = RobotData(position=np.array([1, -1]))
redRobotData = RobotData()
sendChannel, receiveChannel = 0, 1

stateMachine = None  # type: LogicStateMachine

def setup():
    global robot, stateMachine
    robot = Robot(controller.Robot(), blueRobotData)
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    stateMachine = LogicStateMachine(robot)

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

if __name__ == '__main__':
    setup()
    stateMachine.queue((LogicCommand.TRAVEL, (-1, 1)))
    while robot.step():
        process_radio_signals()
        update()
        stateMachine.update_logic()
        stateMachine.update_movement()
        broadcast_update()
