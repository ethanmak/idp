from .utils import *
import queue
from .robot import Robot
from .command import *

class MovementStateMachine:
    def __init__(self, robot: Robot):
        self.movementQueue = queue.Queue()
        self.currentMovementState = None
        self.robot = robot
        self.target = None
        
    def reset_state(self):
        self.currentMovementState = None

    def update(self):
        if self.currentMovementState is None and not self.movementQueue.empty():
            command = self.movementQueue.get()
            self.currentMovementState = command[0]
            self.target = command[1]

            # these sets of if statements are use for movement commands that must be executed only once
            if self.currentMovementState == RobotCommand.FORWARD:
                self.robot.go_forward_distance(self.target)
                return
            elif self.currentMovementState == RobotCommand.SWEEP:
                self.robot.turn_degrees(360, 0.1)
                return
            elif self.currentMovementState == RobotCommand.TURN:
                self.robot.turn_degrees(self.target)
                return

        if self.currentMovementState == RobotCommand.TURN:
            if not self.robot.is_moving():
                print('Finished TURN command to {}'.format(self.target))
                self.reset_state()
        elif self.currentMovementState == RobotCommand.FORWARD:
            if not self.robot.is_moving():
                print('Finished FORWARD command to {}'.format(self.target))
                self.reset_state()
        elif self.currentMovementState == RobotCommand.POINT:
            if self.robot.move_to_target(self.target):
                print('Finished POINT command to {}'.format(self.target))
                self.reset_state()
        elif self.currentMovementState == RobotCommand.OPEN:
            if not self.robot.is_gate_moving():
                print('Finished OPEN command')
                self.reset_state()
        elif self.currentMovementState == RobotCommand.SWEEP:
            if not self.robot.is_moving():
                print('Finished SWEEP command')
                self.reset_state()

    def queue_empty(self):
        return self.movementQueue.empty()

class LogicStateMachine:
    def __init__(self, robot: Robot):
        self.logicQueue = queue.Queue()
        self.movementStateMachine = MovementStateMachine(robot)
        self.currentLogicState = None
        self.target = None
        
    def movement_queue(self, val):
        self.movementStateMachine.movementQueue.put(val)

    def queue(self, val):
        self.logicQueue.put(val)

    def update_movement(self):
        self.movementStateMachine.update()
    
    def update_logic(self):
        if self.currentLogicState is None and self.movementStateMachine.queue_empty() and not self.logicQueue.empty():
            command = self.logicQueue.get()
            self.currentLogicState = command[0]
            self.target = command[1]
        if self.currentLogicState == LogicCommand.CAPTURE:
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.TRAVEL, (1, 1)))
            self.movement_queue((RobotCommand.CLOSE,))
        elif self.currentLogicState == LogicCommand.SEARCH:
            self.movement_queue((RobotCommand.SWEEP,))
        elif self.currentLogicState == LogicCommand.TRAVEL:
            diff = self.target - self.movementStateMachine.robot.robotData.position
            self.movement_queue((RobotCommand.TURN, vector_degree(diff)))
            self.movement_queue((RobotCommand.POINT, self.target))
        elif self.currentLogicState == LogicCommand.DEPOSIT:
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.FORWARD, - 0.1))
            self.movement_queue((RobotCommand.CLOSE,))
            self.movement_queue((RobotCommand.FORWARD, 0.1))
        elif self.currentLogicState == LogicCommand.TRAVEL_BACK:
            self.movement_queue((RobotCommand.TURN, 100))
            self.movement_queue((RobotCommand.POINT, self.movementStateMachine.robot.depositBox))
        self.currentLogicState = None