#red controller (slave controller)
from robot_controller import *
from robot_controller.logic import RobotStateMachine
from robot_controller.display import FieldDisplay
from robot_controller.field import Field

robot = None  # type: Robot
blueRobotData = RobotData(position=np.array([1, -1]), color=Color.BLUE)
redRobotData = RobotData(position=np.array([1, 1]), color=Color.RED)
sendChannel, receiveChannel = 1, 0

field = Field()

stateMachine = None  # type: RobotStateMachine

waitingForTarget = False

def setup():
    global robot, stateMachine
    robot = Robot(controller.Robot(), redRobotData)
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    robot.depositBox = np.array([1, 1])
    stateMachine = RobotStateMachine(robot, blueRobotData, field)

def process_radio_signals():
    global waitingForTarget
    while robot.radio.has_next():
        string = robot.radio.next().split(':')
        signal = string[0]
        if len(string) > 1:
            data = string[1]
        if signal == '':
            continue
        if signal == 'UPDATE':
            blueRobotData.parse(data)
        if signal == 'FIELD':
            field.parse_additions(data, use_id=True)
        elif signal == 'DELETE':
            field.parse_deletions(data)
        elif signal == 'END':
            print('Program END')
            robot.stop_motors()
            waitingForTarget = True
            stateMachine.reset()
            stateMachine.queue((LogicCommand.TRAVEL_BACK,))
        elif signal == 'TARGET':
            waitingForTarget = False
            robot.robotData.targetBlock = int(data)
            block_pos = field.get_block_pos(robot.robotData.targetBlock)
            stateMachine.queue((LogicCommand.TRAVEL,
                                block_pos - 0.15 * normalize(block_pos - robot.robotData.position)))
            stateMachine.queue((LogicCommand.COLOR,))
        elif signal == 'SEARCH':
            waitingForTarget = False
            pos = data.split(' ')
            stateMachine.queue((LogicCommand.TRAVEL, (float(pos[0]), float(pos[1]))))
            stateMachine.queue((LogicCommand.SEARCH,))

def broadcast_update():
    global waitingForTarget
    robot.radio.send('UPDATE:' + repr(redRobotData))
    field_changes = field.get_additions(use_id=False)
    if field_changes:
        robot.radio.send('FIELD:' + field_changes)
    color_changes = field.get_color_changes()
    if color_changes:
        robot.radio.send('COLOR:' + color_changes)
    deletions = field.get_deletions()
    if deletions:
        robot.radio.send('DELETE:' + deletions)
    if stateMachine.currentLogicState is None and not waitingForTarget and stateMachine.movement_command_empty():
        waitingForTarget = True
        robot.radio.send('DONE')

if __name__ == '__main__':
    setup()
    robot.stop_motors()
    stateMachine.queue((LogicCommand.DELAY, 1))
    stateMachine.queue((LogicCommand.SEARCH, False))
    while robot.step():
        process_radio_signals()
        robot.update()
        stateMachine.update_logic()
        stateMachine.check_failsafes(give_way=True)
        stateMachine.update_movement()
        broadcast_update()
