import controller
from robot_controller import *
robot = None  # type: Robot

def setup():
    global robot
    robot = Robot(controller.Robot())
    robot.init_motor_velocity_control()

def exit():
    pass

if __name__ == '__main__':
    setup()
    robot.leftMotor.setPosition(2 * np.pi)
    robot.rightMotor.setPosition(2 * np.pi)
    robot.set_motor_velocity(5, 5)
    initial_pos = np.array([1, 0.04, -1])
    while robot.step():
        if not robot.is_moving() and initial_pos is not None:
            print(np.linalg.norm(robot.get_gps_position() - initial_pos))
            initial_pos = None
        elif initial_pos is not None:
            print(robot.leftMotor.getTorqueFeedback())
        pass
    exit()
