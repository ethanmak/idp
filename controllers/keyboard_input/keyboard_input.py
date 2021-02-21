"""
Keyboard Controller

This controller is used for debugging and allows an easy method to control a robot using keyboard input. It also serves as a test for sensors.
"""
from robot_controller import *
from robot_controller.display import FieldDisplay
from robot_controller.field import Field
import pygame

robot = None  # type: Robot
robotData = RobotData(position=np.array([1, -1]))

field = Field()

fieldDisplay = None  # type: FieldDisplay

def setup():
    global robot, fieldDisplay
    robot = Robot(controller.Robot(), robotData)
    robot.stop_motors()
    robot.init_motor_velocity_control()
    fieldDisplay = FieldDisplay(resolution=800)

if __name__ == '__main__':
    setup()
    while robot.step():
        robot.update()
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            robot.set_motor_velocity(-2, 2)
        elif keys[pygame.K_RIGHT]:
            robot.set_motor_velocity(2, -2)
        elif keys[pygame.K_UP]:
            robot.set_motor_velocity(5, 5)
        elif keys[pygame.K_DOWN]:
            robot.set_motor_velocity(-5, -5)
        else:
            robot.stop_motors()
        fieldDisplay.draw(robotData, None, field) #note that the robot will be blue
        print(Color.get_color(robot.get_color_sensor_value()))
    fieldDisplay.exit()
