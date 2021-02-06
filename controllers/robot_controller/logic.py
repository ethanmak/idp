from .utils import *
import queue
from .robot import Robot
from .command import *
from .field import Field, Color

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
        self.storedState = None
        
    def movement_queue(self, val):
        self.movementQueue.put(val)

    def queue(self, val):
        self.logicQueue.put(val)

    def update_movement(self):
        if self.currentMovementState is None and not self.movementQueue.empty() and self.storedState is None:
            command = self.movementQueue.get()
            self.currentMovementState = command[0]
            if len(command) > 1:
                self.target = command[1]
            else:
                self.target = None

            # these sets of if statements are use for movement commands that must be executed only once
            if self.currentMovementState == RobotCommand.SWEEP:
                self.robot.init_motor_velocity_control()
                self.prev_state = self.robot.robotData.yaw
                self.robot.set_motor_velocity(1.5, -1.5)

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
            rotation = self.robot.robotData.yaw
            wall_dist = Field.distance_to_wall(self.robot.robotData.position, rotation) - 0.1 # accounting for robot depth
            if wall_dist - dist - 0.1 > 0.02 and dist < 1.5 and dist < wall_dist:
                dist += 0.025 + 0.1
                rotation += 10
                angle = np.radians(rotation)
                point = np.array([self.robot.robotData.position[0] + dist * np.cos(angle),
                                  self.robot.robotData.position[1] + dist * np.sin(angle)])
                # print('possible block at ', point)
                if not self.field.contains_point(point, threshold=0.05 * 1.5) and Field.in_bounds(point):
                    # print('found block at {} with dist {} at yaw {} and wall dist {}'.format(point, dist, self.robot.robotData.yaw, wall_dist))
                    self.field.add_block(point, use_field=self.target)
            if self.robot.robotData.yaw < self.prev_state - 0.05 and self.prev_state - self.robot.robotData.yaw < 1:
                print('Finished SWEEP command')
                self.robot.stop_motors()
                self.reset_movement_state()

    def reset_movement_state(self):
        self.currentMovementState = None

    def movement_command_empty(self):
        return self.movementQueue.empty() and self.currentMovementState is None
    
    def update_logic(self):
        target = None
        state = None
        if self.movement_command_empty():
            if self.logicQueue.empty():
                self.currentLogicState = None
                return
            command = self.logicQueue.get()
            state = command[0]
            self.currentLogicState = state
            if len(command) > 1:
                target = command[1]
        if state == LogicCommand.CAPTURE:
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.TRAVEL, target))
            self.movement_queue((RobotCommand.CLOSE,))
        elif state == LogicCommand.SEARCH:
            if target is None:
                target = True
            self.movement_queue((RobotCommand.SWEEP, target))
        elif state == LogicCommand.TRAVEL:
            diff = vector_degree(target - self.robot.robotData.position)
            self.movement_queue((RobotCommand.TURN, diff))
            self.movement_queue((RobotCommand.POINT, target))
        elif state == LogicCommand.DEPOSIT:
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.FORWARD, - 0.1))
            self.movement_queue((RobotCommand.CLOSE,))
            self.movement_queue((RobotCommand.FORWARD, 0.1))
        elif state == LogicCommand.TRAVEL_BACK:
            diff = vector_degree(self.robot.depositBox - self.robot.robotData.position)
            self.movement_queue((RobotCommand.TURN, diff))
            self.movement_queue((RobotCommand.POINT, self.robot.depositBox))
        elif state == LogicCommand.COLOR:
            color = Color.get_color(self.robot.get_color_sensor_value())
            self.field.set_block_color(self.robot.robotData.targetBlock, color)
            if color == self.robot.color:
                self.queue((LogicCommand.CAPTURE,))
                self.queue((LogicCommand.TRAVEL_BACK,))
                self.queue((LogicCommand.DEPOSIT,))
                #determine where to search
            else:
                print('not right color')
                self.movement_queue((RobotCommand.TURN, self.robot.robotData.yaw + 180))
                self.movement_queue((RobotCommand.FORWARD, 0.1))
                self.queue((LogicCommand.SEARCH, True if self.robot.color == Color.BLUE else False))

    def check_failsafes(self, otherRobot):
        dist = np.linalg.norm(self.robot.robotData.position - otherRobot.position)
        if dist < 0.26:
            pass


    def exit(self):
        self.currentLogicState = None
        self.currentMovementState = None
        while not self.movementQueue.empty():
            self.movementQueue.get()
        while not self.logicQueue.empty():
            self.logicQueue.get()
