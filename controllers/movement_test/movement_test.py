"""
Movement Test

A very simple controller used to test the individual sensors, motors, and functions of the Robot class
"""
from robot_controller import *
robot = None  # type: Robot

def setup():
    global robot
    robot = Robot(controller.Robot(), RobotData())
    robot.init_motor_velocity_control()

def exit():
    pass

if __name__ == '__main__':
    setup()
    robot.stop_motors()
    robot.open_gate(True)
    while robot.step():
        robot.update()
        print(robot.get_color_sensor_value(), robot.get_distance())
        pass
    exit()
