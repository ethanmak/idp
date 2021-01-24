import controller
from robot_controller import *
robot = None # type: Robot

def setup():
    global robot
    robot = Robot(controller.Robot())
    robot.init_motor_velocity_control()

def exit():
    pass

if __name__ == '__main__':
    setup()
    while robot.step():
        robot.set_motor_velocity(1, -1)
        print(robot.get_color_sensor_value())
    exit()
