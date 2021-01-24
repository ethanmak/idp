import controller
from .utils import *
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

    def stop_motors(self):
        self.set_motor_velocity(0, 0)

    def get_gps_position(self):
        return self.gps.getValues()

    def get_gps_speed(self):
        return self.gps.getSpeed()

    def get_color_sensor_value(self):
        return tuple(np.array(self.colorSensor.getImageArray()).flatten())

    def go_forward_distance(self, distance):
        pass

    def turn_degrees(self, degree):
        pass

    def is_moving(self):
        return abs(self.leftMotor.getVelocity()) > 0.001 or abs(self.rightMotor.getVelocity()) > 0.001

    def is_gate_moving(self):
        return False #change when gate is added

