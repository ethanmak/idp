# blue controller (master controller)
from robot_controller import *
from robot_controller.logic import RobotStateMachine
from robot_controller.display import FieldDisplay
from robot_controller.field import Field

robot = None  # type: Robot
blueRobotData = RobotData(position=np.array([1, -1]), color=Color.BLUE)
redRobotData = RobotData(position=np.array([1, 1]), color=Color.RED)
sendChannel, receiveChannel = 0, 1

field = Field()

stateMachine = None  # type: RobotStateMachine

fieldDisplay = None  # type: FieldDisplay

end = False
redEnd = False
setRedTarget = False

def setup():
    global robot, stateMachine, fieldDisplay
    robot = Robot(controller.Robot(), blueRobotData)
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    robot.depositBox = np.array([1, -1])

    stateMachine = RobotStateMachine(robot, redRobotData, field)

    fieldDisplay = FieldDisplay(resolution=800, title='Blue Robot Field')


def process_radio_signals():
    global setRedTarget
    while robot.radio.has_next():
        string = robot.radio.next().split(':')
        signal = string[0]
        if len(string) > 1:
            data = string[1]
        if signal == 'UPDATE':
            redRobotData.parse(data)
        elif signal == 'COLOR':
            field.parse_color_changes(data)
        elif signal == 'FIELD':
            field.parse(data, use_id=False, mark_changes=True, threshold=0.05 * 1.5)
        elif signal == 'COLOR':
            field.parse_color_changes(data)
        elif signal == 'DELETE':
            if redRobotData.targetBlock in field.parse_deletions(data):
                print('deleted')
                redRobotData.targetBlock = -1
        elif signal == 'DONE':
            redRobotData.targetBlock = -1
            setRedTarget = True


def broadcast_update():
    robot.radio.send('UPDATE:' + repr(blueRobotData))
    field_changes = field.get_additions(use_id=True)
    if field_changes:
        robot.radio.send('FIELD:' + field_changes)
    deletions = field.get_deletions()
    if deletions:
        robot.radio.send('DELETE:' + deletions)

def set_new_targets():
    global end, setRedTarget, redEnd
    #set blue target
    if stateMachine.currentLogicState is None:
        robot.robotData.targetBlock = field.allocate_block(blueRobotData, redRobotData)
        if robot.robotData.targetBlock >= 0:
            block_pos = field.get_block_pos(robot.robotData.targetBlock)
            block_color = field.get_block_color(robot.robotData.targetBlock)
            target = block_pos - 0.17 * normalize(block_pos - robot.robotData.position)
            stateMachine.queue((LogicCommand.TRAVEL, target))
            if block_color == Color.UNKNOWN:
                stateMachine.queue((LogicCommand.COLOR,))
            else:
                stateMachine.queue((LogicCommand.CAPTURE,))
                stateMachine.queue((LogicCommand.TRAVEL_BACK,))
                stateMachine.queue((LogicCommand.DEPOSIT,))
        else:
            search_position = field.allocate_search(robot.robotData)
            if search_position is None:
                if not end:
                    stateMachine.queue((LogicCommand.TRAVEL_BACK,))
                    end = True
            else:
                stateMachine.queue((LogicCommand.TRAVEL, search_position))
                stateMachine.queue((LogicCommand.SEARCH, True))

    if setRedTarget:
        block_id = field.allocate_block(redRobotData, blueRobotData)
        if block_id >= 0:
            redRobotData.targetBlock = block_id
            robot.radio.send('TARGET:' + str(block_id))
            setRedTarget = False
        else:
            search_pos = field.allocate_search(redRobotData)
            if search_pos is not None:
                robot.radio.send('SEARCH:' + ' '.join(map(str, search_pos)))
                setRedTarget = False
            elif block_id == -1 and not redEnd:
                robot.radio.send('END')
                redEnd = True

if __name__ == '__main__':
    setup()
    robot.stop_motors()
    stateMachine.queue((LogicCommand.DELAY, 1))
    stateMachine.queue((LogicCommand.SEARCH, True))
    while robot.step():
        process_radio_signals()
        robot.update()
        stateMachine.update_logic()
        stateMachine.check_failsafes(give_way=False)
        stateMachine.update_movement()
        broadcast_update()
        set_new_targets()
        fieldDisplay.draw(blueRobotData, redRobotData, field)
    fieldDisplay.exit()
