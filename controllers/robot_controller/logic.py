from .utils import *
import queue
from collections import deque
from .robot import Robot
from .command import *
from .field import Field, Color

_blueDeposit = np.array([])
_redDeposit = np.array([])


class RobotStateMachine:
    def __init__(self, robot: Robot, otherRobotData, field: Field = Field()):
        self.logicQueue = queue.Queue()
        self.movementQueue = deque()
        self.currentMovementState = None
        self.currentLogicState = None
        self.target = None
        self.prev_state = None
        self.robot = robot
        self.field = field
        self.otherRobotData = otherRobotData
        self.prev_time = 0
        
    def movement_queue(self, val):
        self.movementQueue.append(val)

    def queue(self, val):
        self.logicQueue.put(val)

    def update_movement(self):
        if self.currentMovementState is None and len(self.movementQueue) > 0:
            command = self.movementQueue.popleft()
            self.currentMovementState = command[0]
            if len(command) > 1:
                self.target = command[1]
            else:
                self.target = None
            self.prev_time = self.robot.get_simulation_time()

            # these sets of if statements are use for movement commands that must be executed only once
            if self.currentMovementState == RobotCommand.SWEEP:
                self.robot.init_motor_velocity_control()
                self.prev_state = self.robot.robotData.yaw
                self.robot.set_motor_velocity(2, -2)

        if self.currentMovementState == RobotCommand.TURN:
            if self.robot.turn_degrees(self.target) or self.timed_out():
                print('Finished TURN command to {}'.format(self.target))
                self.robot.stop_motors()
                self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.FORWARD:
            self.robot.set_motor_velocity(3, 3)
            if self.robot.get_simulation_time() - self.prev_time >= self.target:
                print('Finished FORWARD Command for {} seconds'.format(self.target))
                self.robot.stop_motors()
                self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.BACKWARD:
            self.robot.set_motor_velocity(-3, -3)
            if self.robot.get_simulation_time() - self.prev_time >= self.target:
                print('Finished BACKWARD Command for {} seconds'.format(self.target))
                self.robot.stop_motors()
                self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.POINT:
            if self.robot.move_to_target(self.target) or self.timed_out():
                print('Finished POINT command to {}'.format(self.target))
                self.robot.stop_motors()
                self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.OPEN:
            self.robot.open_gate(True)
            print('Finished OPEN command')
            self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.CLOSE:
            self.robot.open_gate(False)
            print('Finished CLOSE command')
            self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.SWEEP:
            robot_dist = 0.07
            dist = self.robot.get_distance()
            rotation = self.robot.robotData.yaw
            wall_dist = Field.distance_to_wall(self.robot.robotData.position, rotation) - robot_dist # accounting for robot depth
            if wall_dist - dist > 0.02 and dist < 1.5 and dist < wall_dist:
                dist += 0.025 + robot_dist
                rotation += 1
                angle = np.radians(rotation)
                point = np.array([self.robot.robotData.position[0] + dist * np.cos(angle),
                                  self.robot.robotData.position[1] + dist * np.sin(angle)])
                # print('possible block at ', point)
                if not self.field.contains_point(point, threshold=0.05 * 1.7) and Field.in_bounds(point) and np.linalg.norm(point - self.otherRobotData.position) > 0.154 and not Field.in_deposit_boxes(point):
                    # print('found block at {} with dist {} at yaw {} and wall dist {}'.format(point, dist, self.robot.robotData.yaw, wall_dist))
                    self.field.add_block(point, use_field=self.target)
            if self.robot.robotData.yaw < self.prev_state - 0.05 and self.prev_state - self.robot.robotData.yaw < 1:
                self.prev_state = None
                print('Finished SWEEP command')
                self.robot.stop_motors()
                self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.PAUSE:
            self.robot.stop_motors()
        elif self.currentMovementState == RobotCommand.DELAY:
            self.robot.stop_motors()
            if self.robot.get_simulation_time() - self.prev_time >= self.target:
                print('Finished DELAY command for {} seconds'.format(self.target))
                self.reset_movement_state()

    def timed_out(self):
        timed = self.robot.get_simulation_time() - self.prev_time > 12
        if timed:
            print('Timed Out')
        return timed

    def reset_movement_state(self):
        self.currentMovementState = None

    def movement_command_empty(self):
        return len(self.movementQueue) <= 0 and self.currentMovementState is None
    
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
            self.movement_queue((RobotCommand.BACKWARD, 1))
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.FORWARD, 1.5))
            self.movement_queue((RobotCommand.CLOSE,))
        elif state == LogicCommand.SEARCH:
            if target is None:
                target = True if self.robot.robotData.color == Color.BLUE else False
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.SWEEP, target))
            self.movement_queue((RobotCommand.CLOSE,))
        elif state == LogicCommand.TRAVEL:
            diff = vector_degree(target - self.robot.robotData.position)
            self.movement_queue((RobotCommand.TURN, diff))
            self.movement_queue((RobotCommand.POINT, target))
        elif state == LogicCommand.DEPOSIT:
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.BACKWARD, 1.5))
            self.movement_queue((RobotCommand.CLOSE,))
            self.movement_queue((RobotCommand.FORWARD,1.5))
            self.movement_queue((RobotCommand.BACKWARD, 1))
        elif state == LogicCommand.TRAVEL_BACK:
            diff = vector_degree(self.robot.depositBox - self.robot.robotData.position)
            self.movement_queue((RobotCommand.TURN, diff))
            self.movement_queue((RobotCommand.POINT, self.robot.depositBox))
        elif state == LogicCommand.COLOR:
            value = self.robot.get_color_sensor_value()
            print('RGB is', value)
            color = Color.get_color(value)
            self.field.set_block_color(self.robot.robotData.targetBlock, color)
            if color == self.robot.robotData.color:
                self.queue((LogicCommand.CAPTURE,))
                self.queue((LogicCommand.TRAVEL_BACK,))
                self.queue((LogicCommand.DEPOSIT,))
                self.field.remove_block(self.robot.robotData.targetBlock)
                print(self.robot.robotData.targetBlock, 'block is deposited')
            else:
                self.movement_queue((RobotCommand.BACKWARD, 1.2))
                self.queue((LogicCommand.SEARCH,))
                if color == Color.GREEN or color == Color.UNKNOWN:
                    self.field.remove_block(self.robot.robotData.targetBlock)
            self.robot.robotData.targetBlock = -1

    def check_failsafes(self, check_robot_proximity=False):
        #check distance to other robot
        if check_robot_proximity:
            dist = np.linalg.norm(self.robot.robotData.position - self.otherRobotData.position)
            if dist < 0.12:
                return

        dir = degree_to_vector(self.robot.robotData.yaw)
        start = self.robot.robotData.position + dir * 0.07
        end = start + dir * 0.05

        if self.currentMovementState != RobotCommand.POINT and self.currentMovementState != RobotCommand.PAUSE:
            return

        #check for frontal distance to other robot
        if distance_segment_point(start, end, self.otherRobotData.position) < 0.0779 + 0.0779 * 1.4:
            if self.currentMovementState != RobotCommand.PAUSE:
                self.movementQueue.appendleft((self.currentMovementState, self.target))
                self.currentMovementState = RobotCommand.PAUSE
        elif self.currentMovementState == RobotCommand.PAUSE:
            self.reset_movement_state()

        #check for distance to blocks
        # for id in self.field.field:
        #     if id == self.robot.robotData.targetBlock:
        #         continue
        #     if distance_segment_point(start, end, self.field.field[id][0]) < 0.0779 + 0.025:
        #         print('block collision', id)
        #         self.movementQueue.appendleft((self.currentMovementState, self.target))
        #         self.currentMovementState = None
        #
        #         self.movementQueue.appendleft((RobotCommand.FORWARD, 1))
        #         self.movementQueue.appendleft((RobotCommand.TURN, self.robot.robotData.yaw))
        #         self.movementQueue.appendleft((RobotCommand.FORWARD, 1))
        #         left_dist = Field.distance_to_wall(self.robot.robotData.position, self.robot.robotData.yaw - 90)
        #         right_dist = Field.distance_to_wall(self.robot.robotData.position, self.robot.robotData.yaw + 90)
        #         if left_dist > right_dist:
        #             print('LEFT', self.robot.robotData.yaw - 90)
        #             self.movementQueue.appendleft((RobotCommand.TURN, self.robot.robotData.yaw - 90))
        #         else:
        #             print('RIGHT')
        #             self.movementQueue.appendleft((RobotCommand.TURN, self.robot.robotData.yaw + 90))
        #         return


    def exit(self):
        self.currentLogicState = None
        self.currentMovementState = None
        self.movementQueue.clear()
        while not self.logicQueue.empty():
            self.logicQueue.get()
