import controller
from .field import Color
from .utils import *
from .pid import PIDController
import numpy as np

class Radio:
    def __init__(self, emmitter: controller.Emitter, receiver: controller.Receiver):
        self.sender = emmitter  # type: controller.Emitter
        self.receiver = receiver  # type: controller.Receiver

    def enable(self, timestep):
        self.receiver.enable(timestep)

    def send(self, data: str):
        data = data.encode('utf-8')
        self.sender.send(data)

    def has_next(self):
        return self.receiver.getQueueLength() > 0

    def next(self):
        data = self.receiver.getData()
        if data is None:
            print('data collected was invalid')
            return ''
        self.receiver.nextPacket()
        return data.decode('utf-8')

    def getQueueLength(self):
        return self.receiver.getQueueLength()


class RobotData:
    def __init__(self, position=np.array([0, 0]), color=Color.UNKNOWN):
        self.position = position
        self.yaw = 0
        self.velocity = np.array([0, 0])
        self.targetBlock = -1
        self.color = color
        self.targetPos = None

    def __repr__(self):
        return ' '.join(map(str, self.position)) + ' ' + str(self.yaw)

    def __str__(self):
        return 'Pos: (' + ' '.join(map(str, self.position)) + ') Rot: ' + str(self.yaw)

    def parse(self, data):
        nums = [float(x) for x in data.split()]
        self.position = np.array(nums[:2])
        self.yaw = nums[-1]

class Robot:
    def __init__(self, robot: controller.Robot, robotData: RobotData):
        self._robot = robot  # type: controller.Robot
        self._timestep = int(robot.getBasicTimeStep())
        self.robotData = robotData

        self.leftMotor = robot.getDevice('wheel1')  # type: controller.Motor
        self.rightMotor = robot.getDevice('wheel2')  # type: controller.Motor
        self.leftGate = robot.getDevice('gate_left')  # type: controller.Motor
        self.rightGate = robot.getDevice('gate_right')  # type: controller.Motor
        self._maxMotorVelocity = self.leftMotor.getMaxVelocity()
        self._motorVelocityControl = False

        self.distanceLong = robot.getDevice('ds_long')  # type: controller.DistanceSensor
        self.colorSensor = robot.getDevice('camera')  # type: controller.Camera
        self.gps = robot.getDevice('gps')  # type: controller.GPS
        self.radio = Radio(robot.getDevice('emitter'), robot.getDevice('receiver'))
        self.compass = robot.getDevice('compass')  # type: controller.Compass

        self.leftMotor.enableForceFeedback(self._timestep)
        self.rightMotor.enableForceFeedback(self._timestep)

        self.distanceLong.enable(self._timestep)
        self.colorSensor.enable(self._timestep)
        self.gps.enable(self._timestep)
        self.radio.enable(self._timestep)
        self.compass.enable(self._timestep)

        self._turnController = PIDController(0.02, 0.001, 0.05, 0, 2)
        self._pathController = PIDController(0.01, 0, 0, 0, 3)
        self._pointController = PIDController(1 / 0.1, 0, 0, 0, 0)

        self.depositBox = None

    def init_motor_velocity_control(self):
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self._motorVelocityControl = True

    def update(self):
        pos = self.get_gps_position()
        self.robotData.velocity = (pos - self.robotData.position) / self._timestep * 1e3
        self.robotData.position = pos
        self.robotData.yaw = (vector_degree(self.compass.getValues(), flip_x=True) + 270) % 360

    def step(self):
        return self._robot.step(self._timestep) != -1

    def get_simulation_time(self):
        return self._robot.getTime()

    def set_motor_velocity(self, left_vel, right_vel):
        self.leftMotor.setVelocity(clamp(left_vel, -self._maxMotorVelocity, self._maxMotorVelocity))
        self.rightMotor.setVelocity(clamp(right_vel, -self._maxMotorVelocity, self._maxMotorVelocity))

    def _set_motor_position(self, left_pos, right_pos):
        left_pos = left_pos + self.leftMotor.getTargetPosition() if not self._motorVelocityControl else left_pos
        right_pos = right_pos + self.rightMotor.getTargetPosition() if not self._motorVelocityControl else right_pos
        self.leftMotor.setPosition(left_pos)
        self.rightMotor.setPosition(right_pos)
        self._motorVelocityControl = False

    def stop_motors(self):
        self.set_motor_velocity(0, 0)

    def get_gps_position(self):
        data = self.gps.getValues()
        return np.array([data[0], data[2]])

    def get_velocity(self):
        return self.robotData.velocity

    def get_color_sensor_value(self):
        return tuple(np.array(self.colorSensor.getImageArray()).flatten())

    def get_distance(self):
        # return 0.7611 * self.distanceLong.getValue()**-0.9313 - 0.1252
        return self.distanceLong.getValue() / 1000

    def turn_degrees(self, degree, target_vel=None, threshold=0.5):
        '''
        Turns robot counterclockwise by the degree amount
        :param degree: degrees to turn
        :param target_vel: velocity to turn (default value is self._targetMotorVel)
        :param threshold: the threshold at which the program deems the robot is close enough to its intended orientation
        :return: None
        '''
        if target_vel is None:
            target_vel = self._maxMotorVelocity * 0.8
        # dist = (degree - self.robotData.yaw) / 360 * np.pi * self._wheelbase / self._wheelRadius
        # self._set_motor_position(dist, -dist)
        if not self._motorVelocityControl:
            self.init_motor_velocity_control()
        diff = angle_subtract(degree, self.robotData.yaw)
        turn_vel = target_vel * clamp(self._turnController.update(-diff), -1, 1)
        self.set_motor_velocity(turn_vel, -turn_vel)
        return abs(diff) < threshold

    def is_moving(self, thresh=3e-6):
        return abs(self.leftMotor.getTorqueFeedback()) > thresh or abs(self.rightMotor.getTorqueFeedback()) > thresh

    def move_to_target(self, target, threshold=0.02):
        if not self._motorVelocityControl:
            self.init_motor_velocity_control()
        velocity = self.get_velocity()
        diff = target - self.robotData.position
        dist = np.linalg.norm(diff)
        diff_ang = angle_subtract(vector_degree(diff), self.robotData.yaw)
        if np.all(np.isclose(velocity, [0, 0])):
            turnOutput = 0
        else:
            turnOutput = self._pathController.update(diff_ang)
        motorPower = self._maxMotorVelocity * 0.8* clamp(self._pointController.update(-dist), -1, 1)
        turnOutput *= motorPower
        self.set_motor_velocity(motorPower - turnOutput, motorPower + turnOutput)
        return dist < threshold

    def open_gate(self, is_open: bool):
        if is_open:
            self.leftGate.setPosition(2.35619)
            self.rightGate.setPosition(-2.35619)
        else:
            self.leftGate.setPosition(0)
            self.rightGate.setPosition(0)

