from .utils import *
import queue
from .robot import Robot
from .command import *
from .field import Field

_blueDeposit = np.array([])
_redDeposit = np.array([])


class RobotStateMachine:
    def __init__(self, robot: Robot, field: Field = Field()):
        self.logicQueue = queue.Queue()
        self.movementQueue = queue.Queue()
        self.currentMovementState = None
        self.currentLogicState = None
        self.target = None
        self.prev_state = None
        self.robot = robot
        self.field = field
        
    def movement_queue(self, val):
        self.movementQueue.put(val)

    def queue(self, val):
        self.logicQueue.put(val)

    def update_movement(self):
        if self.currentMovementState is None and not self.movementQueue.empty():
            command = self.movementQueue.get()
            self.currentMovementState = command[0]
            self.target = command[1]

            # these sets of if statements are use for movement commands that must be executed only once
            if self.currentMovementState == RobotCommand.SWEEP:
                self.robot.init_motor_velocity_control()
                self.prev_state = self.robot.robotData.yaw
                self.robot.set_motor_velocity(0.5, -0.5)

        if self.currentMovementState == RobotCommand.TURN:
            if self.robot.turn_degrees(self.target):
                print('Finished TURN command to {}'.format(self.target))
                self.robot.stop_motors()
                self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.FORWARD:
            delta = degree_to_vector(self.robot.robotData.yaw) * self.target
            print(delta)
            self.target = self.robot.robotData.position + delta
            self.currentMovementState = RobotCommand.POINT
            print('Converted FORWARD Command to POINT command to {}'.format(self.target))
        elif self.currentMovementState == RobotCommand.POINT:
            if self.robot.move_to_target(self.target):
                print('Finished POINT command to {}'.format(self.target))
                self.robot.stop_motors()
                self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.OPEN:
            if not self.robot.is_gate_moving():
                print('Finished OPEN command')
                self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.SWEEP:
            dist = self.robot.get_distance()
            if Field.distance_to_wall(self.robot.robotData.position, self.robot.robotData.yaw) - dist > 0.02:
                self.field.add_block(np.array([self.robot.robotData.position[0] + dist * np.cos(self.robot.robotData.yaw), self.robot.robotData.position[1] + dist * np.sin(self.robot.robotData.yaw)]))
            if self.robot.robotData.yaw < self.prev_state and self.prev_state - self.robot.robotData.yaw < 0.5:
                print('Finished SWEEP command')
                self.reset_movement_state()

    def reset_movement_state(self):
        self.currentMovementState = None

    def has_movement_command(self):
        return self.movementQueue.empty()
    
    def update_logic(self):
        target = None
        if self.has_movement_command() and not self.logicQueue.empty():
            command = self.logicQueue.get()
            self.currentLogicState = command[0]
            target = command[1]
        if self.currentLogicState == LogicCommand.CAPTURE:
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.TRAVEL, target))
            self.movement_queue((RobotCommand.CLOSE,))
        elif self.currentLogicState == LogicCommand.SEARCH:
            self.movement_queue((RobotCommand.SWEEP,))
        elif self.currentLogicState == LogicCommand.TRAVEL:
            diff = vector_degree(target - self.robot.robotData.position)
            self.movement_queue((RobotCommand.TURN, diff))
            self.movement_queue((RobotCommand.POINT, target))
        elif self.currentLogicState == LogicCommand.DEPOSIT:
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.FORWARD, - 0.1))
            self.movement_queue((RobotCommand.CLOSE,))
            self.movement_queue((RobotCommand.FORWARD, 0.1))
        elif self.currentLogicState == LogicCommand.TRAVEL_BACK:
            self.movement_queue((RobotCommand.TURN, 100))
            self.movement_queue((RobotCommand.POINT, self.robot.depositBox))
        self.currentLogicState = None

    def exit(self):
        self.currentLogicState = None
        self.currentMovementState = None
        while not self.movementQueue.empty():
            self.movementQueue.get()
        while not self.logicQueue.empty():
            self.logicQueue.get()
