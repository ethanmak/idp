import controller
from .field import Color
from .utils import *
from .pid import PIDController
import numpy as np


class Radio:
    """
    This class handles all the robot radio communication (sending and receiving).
    """

    def __init__(self, emmitter: controller.Emitter, receiver: controller.Receiver):
        """
        Initializer
        :param emmitter: Emitter device from Webots API
        :param receiver: Receiver device from Webbots API
        """
        self.sender = emmitter  # type: controller.Emitter
        self.receiver = receiver  # type: controller.Receiver

    def enable(self, timestep: int) -> None:
        """
        Starts the radio sampling on given timestep

        :param timestep: Timestep between samples
        :return: None
        """
        self.receiver.enable(timestep)

    def send(self, data: str) -> None:
        """
        Sends data across radio send channel

        :param data: String to send over radio
        :return: None
        """
        data = data.encode('utf-8')
        self.sender.send(data)

    def has_next(self) -> bool:
        """
        Returns if receiver has another packet to be processed

        :return: True if receiver has packets in queue, False otherwise
        """
        return self.receiver.getQueueLength() > 0

    def next(self) -> str:
        """
        Returns the next packet in receiver queue and pops it off, if possible

        :return: Next packet if available, otherwise an empty string
        """
        data = self.receiver.getData()
        if data is None:
            print('data collected was invalid')
            return ''
        self.receiver.nextPacket()
        return data.decode('utf-8')

    def getQueueLength(self) -> int:
        """
        Returns the receiver's queue length

        :return: The receiver's queue length
        """
        return self.receiver.getQueueLength()


class RobotData:
    """
    This robot handles all constant and updatable attributes of a robot and meshes that for communication
    """

    def __init__(self, position: np.float = np.array([0, 0]), color: Color = Color.UNKNOWN):
        """
        Initializer
        :param position: Initial position of robot
        :param color: Color of robot
        """
        self.position = position
        self.yaw = 0
        self.velocity = np.array([0, 0])
        self.angularVelocity = 0
        self.targetBlock = -1
        self.color = color
        self.targetPos = None
        self.depositBox = None

    def __repr__(self):
        """
        Returns a parseable string that can be sent over radio

        :return: String with parseable data
        """
        return ' '.join(map(str, self.position)) + ' ' + str(self.yaw) + ' ' + str(self.angularVelocity)

    def __str__(self):
        return 'Pos: (' + ' '.join(map(str, self.position)) + ') Rot: ' + str(self.yaw) + ' Angular Vel: ' + str(self.angularVelocity)

    def parse(self, data) -> None:
        """
        Updates variables based on repr string

        :param data: String with data to be parsed
        :return: None
        """
        nums = [float(x) for x in data.split()]
        self.position = np.array(nums[:2])
        self.yaw = nums[-2]
        self.angularVelocity = nums[-1]


class Robot:
    """
    The class that handles all direct communication and interaction with Webots api and objects
    """

    def __init__(self, robot: controller.Robot, robotData: RobotData):
        """
        Intializer
        :param robot: Webots robot object
        :param robotData: RobotData object to update
        """
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

        self._turnController = PIDController(0.02, 0.002, 0.05, 0, 2)
        self._pathController = PIDController(0.01, 0.002, 0, 0, 3)
        self._pointController = PIDController(1 / 0.1, 0, 0, 0, 0)

        self.depositBox = None

    def init_motor_velocity_control(self) -> None:
        """
        Initializes motors for pure velocity control by setting position to inf

        :return: None
        """
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self._motorVelocityControl = True

    def update(self) -> None:
        """
        Updates robotData with respective values for this timestep

        :return: None
        """
        pos = self.get_gps_position()
        heading = (vector_degree(self.compass.getValues(), flip_x=True) + 270) % 360
        self.robotData.velocity = (pos - self.robotData.position) / self._timestep * 1e3
        self.robotData.angularVelocity = angle_subtract(heading, self.robotData.yaw) / self._timestep * 1e3
        self.robotData.position = pos
        self.robotData.yaw = heading

    def step(self) -> bool:
        """
        Steps physics simulation by basic timestep and returns if simulation should continue

        :return: True if simulation should continue, False if terminated
        """
        return self._robot.step(self._timestep) != -1

    def get_simulation_time(self) -> float:
        """
        Returns the amount of time passed in simulation

        :return: Time passed in simulation
        """
        return self._robot.getTime()

    def set_motor_velocity(self, left_vel, right_vel) -> None:
        """
        Sets the motor velocity of both motors

        :param left_vel: Velocity of left motor
        :param right_vel: Velocity of right motor
        :return: None
        """
        self.leftMotor.setVelocity(clamp(left_vel, -self._maxMotorVelocity, self._maxMotorVelocity))
        self.rightMotor.setVelocity(clamp(right_vel, -self._maxMotorVelocity, self._maxMotorVelocity))

    def _set_motor_position(self, left_pos, right_pos) -> None:
        """
        Sets the target motor position of both motors

        :param left_pos: Target position of left motor
        :param right_pos: Target position of right motor
        :return: None
        """
        left_pos = left_pos + self.leftMotor.getTargetPosition() if not self._motorVelocityControl else left_pos
        right_pos = right_pos + self.rightMotor.getTargetPosition() if not self._motorVelocityControl else right_pos
        self.leftMotor.setPosition(left_pos)
        self.rightMotor.setPosition(right_pos)
        self._motorVelocityControl = False

    def stop_motors(self) -> None:
        """
        Stops the motors of the robot

        :return: None
        """
        self.set_motor_velocity(0, 0)

    def get_gps_position(self) -> np.ndarray:
        """
        Returns the current robot position as measured by the GPS sensor

        :return: X, Z coordinates of GPS (can be regarded as X,Y of a left handed system)
        """
        data = self.gps.getValues()
        return np.array([data[0], data[2]])

    def get_velocity(self) -> np.ndarray:
        """
        Returns velocity calculated by update

        :return: Robot velocity
        """
        return self.robotData.velocity

    def get_color_sensor_value(self) -> tuple:
        """
        Gives rgb value measured by color sensor

        :return: RGB values of color sensor in tuple
        """
        return tuple(np.array(self.colorSensor.getImageArray()).flatten())

    def get_distance(self) -> float:
        """
        Gives distance measured by front sensor

        :return: Distance measured by infrared sensor
        """
        return self.distanceLong.getValue() / 1000

    def turn_degrees(self, degree: float, target_vel: float = None, threshold: float = 0.5) -> bool:
        '''
        Turns robot counterclockwise by the degree amount

        :param degree: Degree heading to turn to
        :param target_vel: Velocity to turn (default value is 0.8 * self._maxMotorVelocity)
        :param threshold: Threshold at which the program deems the robot is close enough to its intended orientation
        :return: True if robot heading is close enough to the intended heading, False otherwise
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

    def is_moving(self, thresh=3e-3) -> bool:
        """
        Returns if robot is currently moving or turning

        :param thresh: Amount by which to determine if robot is moving
        :return: True if robot is moving, False otherwise
        """
        return np.linalg.norm(self.robotData.velocity) > thresh and self.robotData.angularVelocity > thresh

    def move_to_target(self, target, threshold=0.02) -> bool:
        """
        Moves robot to target point using PID heading correction and power control

        :param target: Target position of robot
        :param threshold: Distance to which is considered arrived
        :return: True if distance to point < threshold, False otherwise
        """
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
        # turnOutput = 0
        motorPower = self._maxMotorVelocity * 0.8 * clamp(self._pointController.update(-dist), -1, 1)
        turnOutput *= motorPower
        self.set_motor_velocity(motorPower - turnOutput, motorPower + turnOutput)
        return dist < threshold

    def open_gate(self, is_open: bool) -> None:
        """
        Opens and closes gate

        :param is_open: Bool to determine state of gate: True if open, False if closed
        :return: None
        """
        if is_open:
            self.leftGate.setPosition(2.35619)
            self.rightGate.setPosition(-2.35619)
        else:
            self.leftGate.setPosition(0)
            self.rightGate.setPosition(0)
