"""
Blue Controller (Master controller)

This controller is the main controller executing code on the robot. It interacts with the robot from a very high level, leaving lower level routines to RobotStateMachine
"""
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
    """
    This sets up the simulation and GUI to run

    :return: None
    """
    global robot, stateMachine, fieldDisplay
    robot = Robot(controller.Robot(), blueRobotData)
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    robot.depositBox = np.array([1, -1])

    stateMachine = RobotStateMachine(robot, redRobotData, field)

    fieldDisplay = FieldDisplay(resolution=400, title='Blue Robot Field')


def process_radio_signals():
    """
    Process the incoming signals from the radio and turn them into commands or updates

    Possible signals are:
    UPDATE: Update on the pose of the other robot
    COLOR: Updates on colors of a set of existing blocks
    FIELD: Possible new additions of blocks to the field
    DELETE: Deletions of blocks on field representation
    DONE: The follower robot has finished its task

    :return: None
    """
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
            field.parse_additions(data, use_id=False, mark_changes=True, threshold=0.05 * 1.5)
        elif signal == 'DELETE':
            if redRobotData.targetBlock in field.parse_deletions(data):
                print('deleted')
                redRobotData.targetBlock = -1
        elif signal == 'DONE':
            redRobotData.targetBlock = -1
            setRedTarget = True


def broadcast_update():
    """
    Send an update about the RobotData and field representation to the other robot
    Updates are:
    UPDATE: Update on RobotData (robot pose)
    FIELD: New additions to field representation
    DELETE: New deletions to field representation

    :return: None
    """
    robot.radio.send('UPDATE:' + repr(blueRobotData))
    field_changes = field.get_additions(use_id=True)
    if field_changes:
        robot.radio.send('FIELD:' + field_changes)
    deletions = field.get_deletions()
    if deletions:
        robot.radio.send('DELETE:' + deletions)

def set_new_targets():
    """
    Defines new target blocks or search positions for both robots when they finish their tasks, and queues LogicCommands for master robot

    :return: None
    """
    global end, setRedTarget, redEnd
    #set blue target
    if stateMachine.currentLogicState is None:
        robot.robotData.targetBlock = field.allocate_block(blueRobotData, redRobotData)
        if robot.robotData.targetBlock >= 0:
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
            end = False
        else:
            search_position = field.allocate_search(robot.robotData)
            if search_position is None:
                if not end:
                    stateMachine.queue((LogicCommand.TRAVEL_BACK,))
                    end = True
                    print('Program END')
            else:
                end = False
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
