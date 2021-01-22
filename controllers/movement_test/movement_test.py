import controller
from robot_controller import *
robot = None # type: Robot

def setup():
    global robot
    robot = Robot(controller.Robot())

def exit():
    pass

if __name__ == '__main__':
    print("in main")
    setup()
    while robot.step():
        print("in loop")
        robot.set_motor_velocity(50, 50)
        print(robot.get_gps_position())
    exit()
