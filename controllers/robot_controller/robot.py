import controller


class Robot:
    def __init__(self, robot: controller.Robot):
        self._robot = robot  # type: controller.Robot
        self._timestep = int(robot.getBasicTimeStep())

        self.leftMotor = robot.getDevice('wheel1')  # type: controller.Motor
        self.rightMotor = robot.getDevice('wheel2')  # type: controller.Motor
        self.leftDistance = robot.getDevice('ds_left')  # type: controller.DistanceSensor
        self.rightDistance = robot.getDevice('ds_right')  # type: controller.DistanceSensor
        self.colorSensor = robot.getDevice('camera')  # type: controller.Camera
        self.gps = robot.getDevice('gps')  # type: controller.GPS

        self.leftDistance.enable(self._timestep)
        self.rightDistance.enable(self._timestep)
        self.colorSensor.enable(self._timestep)
        self.gps.enable(self._timestep)

    def step(self):
        return self._robot.step(self._timestep) != -1

    def set_motor_power(self, left_power, right_power):
        self.leftMotor.setTorque(left_power)
        self.rightMotor.setTorque(right_power)

    def set_motor_velocity(self, left_vel, right_vel):
        self.leftMotor.setVelocity(left_vel)
        self.rightMotor.setVelocity(right_vel)

    def get_gps_position(self):
        return self.gps.getValues()

    def get_gps_speed(self):
        return self.gps.getSpeed()

