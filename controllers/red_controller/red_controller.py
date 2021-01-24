#red controller (slave controller)
import controller
from robot_controller import *

robot = None  # type: Robot
blueRobotData = RobotData()
redRobotData = RobotData()
sendChannel, receiveChannel = 1, 0

def setup():
    global robot
    robot = Robot(controller.Robot())
    robot.init_motor_velocity_control()
    robot.stop_motors()
    robot.radio.sender.setChannel(sendChannel)
    robot.radio.receiver.setChannel(receiveChannel)

def exit():
    pass

def update():
    redRobotData.position = robot.get_gps_position()

def process_radio_signals():
    while robot.radio.hasNext():
        data = robot.radio.next()
        if data == '':
            continue
        if data.find('UPDATE:') != -1:
            blueRobotData.parse(data)

def broadcast_update():
    robot.radio.send('UPDATE:' + repr(redRobotData))

if __name__ == '__main__':
    setup()
    while robot.step():
        process_radio_signals()
        update()
        broadcast_update()
    exit()
