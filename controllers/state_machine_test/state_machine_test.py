"""
State Machine Test

A barebones controller to allow the unit testing of LogicCommands and RobotCommands
"""
from robot_controller import *
from robot_controller.logic import RobotStateMachine
from robot_controller.display import FieldDisplay
from robot_controller.field import Field

robot = None  # type: Robot
robotData = RobotData(position=np.array([1, -1]))
sendChannel, receiveChannel = 0, 1

field = Field()

stateMachine = None  # type: RobotStateMachine

fieldDisplay = None  # type: FieldDisplay

def setup():
    global robot, stateMachine, fieldDisplay
    robot = Robot(controller.Robot(), robotData)
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)
    stateMachine = RobotStateMachine(robot, field)

    fieldDisplay = FieldDisplay(resolution=800)

if __name__ == '__main__':
    setup()
    stateMachine.queue((LogicCommand.SEARCH,))
    # stateMachine.queue((LogicCommand.TRAVEL, (-1, 1)))
    # stateMachine.movement_queue((RobotCommand.TURN, 180))
    # stateMachine.movement_queue((RobotCommand.FORWARD, 1))
    while robot.step():
        robot.update()
        stateMachine.update_logic()
        stateMachine.update_movement()
        fieldDisplay.draw(robotData, None, field) #note that the robot will be blue
    fieldDisplay.exit()
