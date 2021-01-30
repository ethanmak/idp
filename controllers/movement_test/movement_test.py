import controller
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
    robot.set_motor_velocity(-1, 1)
    # robot.stop_motors()
    while robot.step():
        robot.update()
        # robot.turn_degrees(90)
        print(robot.robotData.yaw, robot.get_distance())
        pass
    exit()
