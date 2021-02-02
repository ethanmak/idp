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

waitingForTarget = False

def setup():
    global robot, stateMachine, fieldDisplay
    robot = Robot(controller.Robot(), redRobotData, Color.RED)
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    robot.depositBox = np.array([1, 1])
    stateMachine = RobotStateMachine(robot, field)

    fieldDisplay = FieldDisplay(resolution=800, title='Red Robot Field')

def process_radio_signals():
    global waitingForTarget
    while robot.radio.hasNext():
        string = robot.radio.next().split(':')
        signal = string[0]
        if len(string) > 1:
            data = string[1]
        if signal == '':
            continue
        if signal == 'UPDATE':
            blueRobotData.parse(data)
        if signal == 'FIELD':
            field.parse(data, use_id=True)
        elif signal == 'END':
            robot.stop_motors()
            waitingForTarget = False
            stateMachine.exit()
            stateMachine.queue((LogicCommand.TRAVEL_BACK,))
        elif signal == 'TARGET':
            waitingForTarget = False
            robot.robotData.targetBlock = int(data)
            stateMachine.queue((LogicCommand.TRAVEL,
                                add_distance_vector(field.get_block_pos(robot.robotData.targetBlock), -0.2)))
            stateMachine.queue((LogicCommand.COLOR,))

def broadcast_update():
    global waitingForTarget
    robot.radio.send('UPDATE:' + repr(redRobotData))
    field_changes = field.get_additions(use_id=False)
    if field_changes:
        robot.radio.send('FIELD:' + field_changes)
    if stateMachine.currentLogicState is None and not waitingForTarget:
        waitingForTarget = True
        robot.radio.send('DONE')

if __name__ == '__main__':
    setup()
    stateMachine.queue((LogicCommand.SEARCH, False))
    while robot.step():
        process_radio_signals()
        robot.update()
        stateMachine.update_logic()
        stateMachine.update_movement()
        broadcast_update()
        fieldDisplay.draw(blueRobotData, redRobotData, field)
    fieldDisplay.exit()
