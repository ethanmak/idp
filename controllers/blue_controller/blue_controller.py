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
    robot = Robot(controller.Robot(), blueRobotData, Color.BLUE)
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    robot.depositBox = np.array([1, -1])

    stateMachine = RobotStateMachine(robot, field)

    fieldDisplay = FieldDisplay(resolution=800, title='Blue Robot Field')

def process_radio_signals():
    while robot.radio.hasNext():
        string = robot.radio.next().split(':')
        signal = string[0]
        if len(string) > 1:
            data = string[1]
        if signal == 'UPDATE':
            redRobotData.parse(data)
        elif signal == 'COLOR':
            field.parse_color_changes(data)
        elif signal == 'FIELD':
            field.parse(data, use_id=False, mark_changes=True)
        elif signal == 'COLOR':
            field.parse_color_changes(data)
        elif signal == 'DONE':
            print('red robot done')


def broadcast_update():
    robot.radio.send('UPDATE:' + repr(blueRobotData))
    field_changes = field.get_additions(use_id=True)
    if field_changes:
        robot.radio.send('FIELD:' + field_changes)

if __name__ == '__main__':
    setup()
    stateMachine.queue((LogicCommand.SEARCH, True))
    while robot.step():
        process_radio_signals()
        robot.update()
        stateMachine.update_logic()
        stateMachine.update_movement()
        broadcast_update()
        fieldDisplay.draw(blueRobotData, redRobotData, field)
    fieldDisplay.exit()
