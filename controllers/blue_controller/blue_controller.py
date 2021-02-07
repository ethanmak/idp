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

end = False

def setup():
    global robot, stateMachine, fieldDisplay
    robot = Robot(controller.Robot(), blueRobotData, Color.BLUE)
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    robot.depositBox = np.array([1, -1])

    stateMachine = RobotStateMachine(robot, redRobotData, field)

    fieldDisplay = FieldDisplay(resolution=800, title='Blue Robot Field')


def process_radio_signals():
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
            print('color')
        elif signal == 'DONE':
            block_id = field.allocate_block(redRobotData.position, Color.RED)
            if block_id != -1:
                robot.radio.send('TARGET:' + str(block_id))
            else:
                search_pos = field.allocate_search(redRobotData.position)
                if search_pos is not None:
                    robot.radio.send('SEARCH:' + ' '.join(map(str, search_pos)))
                else:
                    robot.radio.send('END')


def broadcast_update():
    robot.radio.send('UPDATE:' + repr(blueRobotData))
    field_changes = field.get_additions(use_id=True)
    if field_changes:
        robot.radio.send('FIELD:' + field_changes)

def set_new_target():
    global end
    if stateMachine.currentLogicState is None:
        robot.robotData.targetBlock = field.allocate_block(blueRobotData.position, Color.BLUE)
        if robot.robotData.targetBlock != -1:
            block_pos = field.get_block_pos(robot.robotData.targetBlock)
            block_color = field.get_block_color(robot.robotData.targetBlock)
            target = block_pos - 0.15 * normalize(block_pos - robot.robotData.position)
            stateMachine.queue((LogicCommand.TRAVEL, target))
            if block_color == Color.UNKNOWN:
                stateMachine.queue((LogicCommand.COLOR,))
            else:
                stateMachine.queue((LogicCommand.CAPTURE,))
                stateMachine.queue((LogicCommand.TRAVEL_BACK,))
                stateMachine.queue((LogicCommand.DEPOSIT,))
        else:
            search_position = field.allocate_search(robot.robotData.position)
            if search_position is None:
                stateMachine.queue((LogicCommand.TRAVEL_BACK,))
            else:
                stateMachine.queue((LogicCommand.TRAVEL, search_position))
                stateMachine.queue((LogicCommand.SEARCH, True))

if __name__ == '__main__':
    setup()
    stateMachine.queue((LogicCommand.SEARCH, True))
    while robot.step():
        process_radio_signals()
        robot.update()
        stateMachine.update_logic()
        stateMachine.update_movement()
        set_new_target()
        broadcast_update()
        fieldDisplay.draw(blueRobotData, redRobotData, field)
    fieldDisplay.exit()
