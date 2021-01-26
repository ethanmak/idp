import controller
from robot_controller import *
robot = None  # type: Robot

def setup():
    global robot
    robot = Robot(controller.Robot(), RobotData())
    robot._init_motor_velocity_control()

def exit():
    pass

if __name__ == '__main__':
    setup()
    robot.turn_degrees(360)
    initial_pos = np.array([1, 0.0392, -1])
    while robot.step():
        if not robot.is_moving() and initial_pos is not None:
            print(np.linalg.norm(robot.get_gps_position() - initial_pos))
            initial_pos = None
        elif initial_pos is not None:
            print(robot.leftMotor.getTorqueFeedback())
        pass
    exit()
