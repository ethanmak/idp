import controller
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

    def hasNext(self):
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
    def __init__(self):
        self.position = np.array([])
        self.rotation = np.zeros((3))

    def __repr__(self):
        return ' '.join(map(str, self.position)) + ' ' + ' '.join(map(str, self.rotation))

    def __str__(self):
        return 'Pos: (' + ' '.join(self.position) + ') Rot: (' + ' '.join(self.rotation) + ')'

    def parse(self, data):
        nums = [float(x) for x in data[7:].split()]
        self.position = np.array(nums[:3])
        self.rotation = np.array(nums[3:])

class Robot:
    def __init__(self, robot: controller.Robot):
        self._robot = robot  # type: controller.Robot
        self._timestep = int(robot.getBasicTimeStep())

        self.leftMotor = robot.getDevice('wheel1')  # type: controller.Motor
        self.rightMotor = robot.getDevice('wheel2')  # type: controller.Motor
        self.leftMotor.enableForceFeedback(self._timestep)
        self.rightMotor.enableForceFeedback(self._timestep)
        self._maxMotorVelocity = self.leftMotor.getMaxVelocity()
        self._motorVelocityControl = False

        self.leftDistance = robot.getDevice('ds_left')  # type: controller.DistanceSensor
        self.rightDistance = robot.getDevice('ds_right')  # type: controller.DistanceSensor
        self.colorSensor = robot.getDevice('camera')  # type: controller.Camera
        self.gps = robot.getDevice('gps')  # type: controller.GPS
        self.radio = Radio(robot.getDevice('emitter'), robot.getDevice('receiver'))

        self.leftDistance.enable(self._timestep)
        self.rightDistance.enable(self._timestep)
        self.colorSensor.enable(self._timestep)
        self.gps.enable(self._timestep)
        self.radio.enable(self._timestep)

        self._pathController = PIDController(1,1,1,1,0.005)
        self._pointController = PIDController(1,1,1,0,0.005)
        self._targetMotorVel = self._maxMotorVelocity / 2
        self._wheelRadius = 0.04
        self._wheelBase = 20

    def init_motor_velocity_control(self):
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self._motorVelocityControl = True

    def step(self):
        return self._robot.step(self._timestep) != -1

    def set_motor_velocity(self, left_vel, right_vel):
        if not self._motorVelocityControl:
            raise ValueError('robot must be initialized to use velocity control before calling this function')
        self.leftMotor.setVelocity(clamp(left_vel, -self._maxMotorVelocity, self._maxMotorVelocity))
        self.rightMotor.setVelocity(clamp(right_vel, -self._maxMotorVelocity, self._maxMotorVelocity))

    def _set_motor_position(self, left_pos, right_pos):
        self.leftMotor.setPosition(left_pos)
        self.rightMotor.setPosition(right_pos)

    def stop_motors(self):
        self.set_motor_velocity(0, 0)

    def get_gps_position(self):
        return np.array(self.gps.getValues())

    def get_gps_speed(self):
        return np.array(self.gps.getSpeed())

    def get_color_sensor_value(self):
        return tuple(np.array(self.colorSensor.getImageArray()).flatten())

    def go_forward_distance(self, distance):
        '''
        Sets target position to run forward or backward for a defined distance

        One rotation goes 0.23875863135588146 in coordinates
        :param distance: distance to travel, positive for forward, negative for backward
        :return: None
        '''
        self._set_motor_position(distance / self._wheelRadius / 2, distance / self._wheelRadius / 2)

    def turn_degrees(self, degree):
        self.turn_degrees(degree, self._targetMotorVel)

    def turn_degrees(self, degree, target_vel):
        self._set_motor_position(degree, -degree)
        self.set_motor_velocity(target_vel, -target_vel)

    def is_moving(self):
        return abs(self.leftMotor.getTorqueFeedback()) > 8e-8 or abs(self.rightMotor.getTorqueFeedback()) > 8e-8

    def is_gate_moving(self):
        return False #change when gate is added

    def move_to_target(self, target, threshold=0.05):
        pos = self.get_gps_position()
        dist = np.linalg.norm(pos - target)
        if not self._motorVelocityControl:
            self.init_motor_velocity_control()
        speed = self.get_gps_speed()
        turnOutput = self._pathController.update(np.dot(normalize(target - pos), speed))
        motorPower = self._targetMotorVel #* clamp(self._pointController.update(dist), -1, 1)
        self.set_motor_velocity(motorPower + turnOutput, motorPower - turnOutput)
        return dist < threshold

    def open_gate(self, is_open: bool):
        if is_open:
            pass
        else:
            pass

