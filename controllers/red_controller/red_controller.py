#red controller (slave controller)
from robot_controller import *
from robot_controller.logic import RobotStateMachine
from robot_controller.display import FieldDisplay
from robot_controller.field import Field

robot = None  # type: Robot
blueRobotData = RobotData(position=np.array([1, -1]))
redRobotData = RobotData(position=np.array([1, 1]))
sendChannel, receiveChannel = 1, 0

field = Field()

stateMachine = None  # type: RobotStateMachine

fieldDisplay = None  # type: FieldDisplay

def setup():
    global robot, stateMachine, fieldDisplay
    robot = Robot(controller.Robot(), redRobotData)
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    stateMachine = RobotStateMachine(robot, field)

    fieldDisplay = FieldDisplay(resolution=800, title='Red Robot Field')

def process_radio_signals():
    while robot.radio.hasNext():
        data = robot.radio.next()
        if data == '':
            continue
        if data.find('UPDATE:') != -1:
            blueRobotData.parse(data)
        if data.find('FIELD') != -1:
            field.parse(data)
        elif data.find('END') != -1:
            robot.stop_motors()
            stateMachine.exit()
            stateMachine.queue((LogicCommand.TRAVEL, (1, 1)))


def broadcast_update():
    robot.radio.send('UPDATE:' + repr(redRobotData))

if __name__ == '__main__':
    setup()
    while robot.step():
        process_radio_signals()
        robot.update()
        stateMachine.update_logic()
        stateMachine.update_movement()
        broadcast_update()
        # print(redRobotData)
        fieldDisplay.draw(blueRobotData, redRobotData, None)
    fieldDisplay.exit()
