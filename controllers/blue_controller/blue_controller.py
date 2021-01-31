# blue controller (master controller)
from robot_controller import *
from robot_controller.logic import RobotStateMachine
from robot_controller.display import FieldDisplay
from robot_controller.field import Field

robot = None  # type: Robot
blueRobotData = RobotData(position=np.array([1, -1]))
redRobotData = RobotData(position=np.array([1, 1]))
sendChannel, receiveChannel = 0, 1

field = Field()

stateMachine = None  # type: RobotStateMachine

fieldDisplay = None  # type: FieldDisplay

def setup():
    global robot, stateMachine, fieldDisplay
    robot = Robot(controller.Robot(), blueRobotData)
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    stateMachine = RobotStateMachine(robot, field)

    fieldDisplay = FieldDisplay(resolution=800, title='Blue Robot Field')

def process_radio_signals():
    while robot.radio.hasNext():
        data = robot.radio.next()
        if data == '':
            continue
        if data.find('UPDATE:') != -1:
            redRobotData.parse(data)
        elif data.find('COLOR') != -1:
            pass
        elif data.find('BLOCK') != -1:
            pass

def broadcast_update():
    robot.radio.send('UPDATE:' + repr(blueRobotData))
    field_changes = field.get_additions()
    if field_changes:
        robot.radio.send('FIELD:' + field_changes)

if __name__ == '__main__':
    setup()
    while robot.step():
        process_radio_signals()
        robot.update()
        stateMachine.update_logic()
        stateMachine.update_movement()
        broadcast_update()
        fieldDisplay.draw(blueRobotData, redRobotData, None)
    fieldDisplay.exit()
