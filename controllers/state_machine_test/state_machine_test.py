# blue controller (master controller)
from robot_controller import *
from robot_controller.logic import RobotStateMachine
from robot_controller.display import FieldDisplay

robot = None  # type: Robot
blueRobotData = RobotData(position=np.array([1, -1]))
redRobotData = RobotData()
sendChannel, receiveChannel = 0, 1

stateMachine = None  # type: RobotStateMachine

fieldDisplay = None  # type: FieldDisplay

def setup():
    global robot, stateMachine, fieldDisplay
    robot = Robot(controller.Robot(), blueRobotData)
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    stateMachine = RobotStateMachine(robot)

    fieldDisplay = FieldDisplay(resolution=800)

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
    stateMachine.movement_queue((RobotCommand.TURN, 180))
    stateMachine.movement_queue((RobotCommand.FORWARD, 1))
    while robot.step():
        process_radio_signals()
        robot.update()
        stateMachine.update_logic()
        stateMachine.update_movement()
        broadcast_update()
        fieldDisplay.draw(blueRobotData, None, None)
    fieldDisplay.exit()
