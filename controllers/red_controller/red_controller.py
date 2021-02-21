"""
Blue Controller (Follower controller)

This controller is the main controller executing code on the robot. It interacts with the robot from a very high level, leaving lower level routines to RobotStateMachine
"""
from robot_controller import *
from robot_controller.logic import RobotStateMachine
from robot_controller.field import Field

robot = None  # type: Robot
blueRobotData = RobotData(position=np.array([1, -1]), color=Color.BLUE)
redRobotData = RobotData(position=np.array([1, 1]), color=Color.RED)
sendChannel, receiveChannel = 1, 0

field = Field()

stateMachine = None  # type: RobotStateMachine

waitingForTarget = False

def setup():
    """
    This sets up the simulation to run

    :return: None
    """
    global robot, stateMachine
    robot = Robot(controller.Robot(), redRobotData)
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    robot.depositBox = np.array([1, 1])
    stateMachine = RobotStateMachine(robot, blueRobotData, field)

def process_radio_signals():
    """
    Process the incoming signals from the radio and turn them into commands or updates

    Possible signals are:
    UPDATE: Update on the pose of the other robot
    FIELD: Possible new additions of blocks to the field
    DELETE: Deletions of blocks on field representation
    END: The master has determined that the follower program should end
    TARGET: The master has decided on a new target block for the follower
    SEARCH: The master has decided on a new position to sweep for the follower

    :return: None
    """
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
    """
    Send an update about the RobotData and field representation to the other robot
    Updates are:
    UPDATE: Update on RobotData (robot pose)
    FIELD: New additions to field representation
    DELETE: New deletions to field representation
    COLOR: New updates on colors of existing blocks
    DONE: The follower robot has finished its tasks

    :return: None
    """
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
