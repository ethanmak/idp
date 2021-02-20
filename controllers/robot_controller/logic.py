from .utils import *
import queue
from collections import deque
from .robot import Robot
from .command import *
from .field import Field, Color
import matplotlib.pyplot as plt

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
        self.data = []
        
    def movement_queue(self, val):
        self.movementQueue.append(val)
        
    def _movement_stack(self, val):
        self.movementQueue.appendleft(val)

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
            print('Started {} command with target {}'.format(self.currentMovementState.name, self.target))

            # these sets of if statements are use for movement commands that must be executed only once
            if self.currentMovementState == RobotCommand.SWEEP:
                self.robot.init_motor_velocity_control()
                self.prev_state = self.robot.robotData.yaw
                self.robot.set_motor_velocity(1.5, -1.5)

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
            robot_dist = 0.01
            dist = self.robot.get_distance()
            self.data.append(dist)
            rotation = self.robot.robotData.yaw
            wall_dist = Field.distance_to_wall(self.robot.robotData.position, rotation) - robot_dist # accounting for robot depth
            wall_dist = clamp(wall_dist, 0, 1.5)
            if wall_dist - dist > 0.02 and 0.06 < dist < 1.5 and dist < wall_dist:
                dist += 0.025 + robot_dist
                rotation += 0.5
                angle = np.radians(rotation)
                point = np.array([self.robot.robotData.position[0] + dist * np.cos(angle),
                                  self.robot.robotData.position[1] + dist * np.sin(angle)])
                # print('possible block at ', point)
                if not self.field.contains_point(point, threshold=0.05 * 1.5) \
                        and Field.in_bounds(point) \
                        and np.linalg.norm(point - self.otherRobotData.position) > 0.2\
                        and not Field.in_deposit_boxes(point):
                    print('found block at {} with dist {} at yaw {} and wall dist {}'.format(point, dist, self.robot.robotData.yaw, wall_dist))
                    self.field.add_block(point, use_field=self.target)
            if self.robot.robotData.yaw < self.prev_state - 0.05 and self.prev_state - self.robot.robotData.yaw < 1:
                # plt.plot(self.data)
                # plt.show()
                self.data.clear()
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
        elif self.currentMovementState == RobotCommand.REMOVE_BLOCK:
            self.field.remove_block(self.target)
            print('Removed block with ID {} from the field'.format(self.target))
            if self.robot.robotData.targetBlock == self.target:
                self.robot.robotData.targetBlock = -1
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
            print('CAPTURE of ', self.robot.robotData.targetBlock)
            self.movement_queue((RobotCommand.BACKWARD, 1))
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.FORWARD, 2.5))
            self.movement_queue((RobotCommand.CLOSE,))
            self.movement_queue((RobotCommand.REMOVE_BLOCK, self.robot.robotData.targetBlock))
        elif state == LogicCommand.SEARCH:
            if target is None:
                target = True if self.robot.robotData.color == Color.BLUE else False
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.DELAY, 0.5))
            self.movement_queue((RobotCommand.SWEEP, target))
            self.movement_queue((RobotCommand.CLOSE,))
        elif state == LogicCommand.TRAVEL:
            diff = vector_degree(target - self.robot.robotData.position)
            self.movement_queue((RobotCommand.TURN, diff))
            self.movement_queue((RobotCommand.POINT, target))
        elif state == LogicCommand.DEPOSIT:
            self.movement_queue((RobotCommand.OPEN,))
            self.movement_queue((RobotCommand.BACKWARD, 1.2))
            self.movement_queue((RobotCommand.CLOSE,))
            # self.movement_queue((RobotCommand.FORWARD, 1.5))
            # self.movement_queue((RobotCommand.BACKWARD, 1))
        elif state == LogicCommand.TRAVEL_BACK:
            diff = vector_degree(self.robot.depositBox - self.robot.robotData.position)
            self.movement_queue((RobotCommand.TURN, diff))
            self.movement_queue((RobotCommand.POINT, self.robot.depositBox))
        elif state == LogicCommand.COLOR:
            value = self.robot.get_color_sensor_value()
            color = Color.get_color(value)
            print('RGB is', value, 'with color', color.name)
            self.field.set_block_color(self.robot.robotData.targetBlock, color)
            if color == Color.GREEN or color == Color.UNKNOWN:
                self.field.remove_block(self.robot.robotData.targetBlock)
            if color == self.robot.robotData.color:
                self.queue((LogicCommand.CAPTURE,))
                self.queue((LogicCommand.TRAVEL_BACK,))
                self.queue((LogicCommand.DEPOSIT,))
            else:
                self.movement_queue((RobotCommand.BACKWARD, 1.2))
                self.queue((LogicCommand.SEARCH,))
                self.robot.robotData.targetBlock = -1
        elif state == LogicCommand.DELAY:
            self.movement_queue((RobotCommand.DELAY, target))
    
    def check_failsafes(self, give_way=False):
        block_search_radius = 0.071
        robot_search_radius = 0.051 * 1.5

        dir = degree_to_vector(self.robot.robotData.yaw)
        start = self.robot.robotData.position + 0.064 * dir
        robot_end = start + dir * robot_search_radius * 1.2
        block_end = start + dir * 0.08

        left_dist = Field.distance_to_wall(self.robot.robotData.position, self.robot.robotData.yaw - 90)
        right_dist = Field.distance_to_wall(self.robot.robotData.position, self.robot.robotData.yaw + 90)

        if self.currentMovementState != RobotCommand.POINT and self.currentMovementState != RobotCommand.PAUSE:
            return

        #check for frontal distance to other robot
        robot_dist = distance_segment_point(start, robot_end, self.otherRobotData.position)
        if robot_dist < robot_search_radius + robot_search_radius * 1.4:
            if abs(self.otherRobotData.yaw - self.robot.robotData.yaw - 180) < 60 and give_way:
                self._movement_stack((self.currentMovementState, self.target))
                self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw))
                self._movement_stack((RobotCommand.FORWARD, 2))
                if left_dist > right_dist:
                    self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw + 90))
                else:
                    self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw - 90))
                self._movement_stack((RobotCommand.FORWARD, 3))
                self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw))
                self._movement_stack((RobotCommand.FORWARD, 2))
                if left_dist > right_dist:
                    self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw - 90))
                else:
                    self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw + 90))
                self.reset_movement_state()
            elif self.currentMovementState != RobotCommand.PAUSE:
                self._movement_stack((self.currentMovementState, self.target))
                self._movement_stack((RobotCommand.PAUSE,))
                self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.PAUSE:
            self.reset_movement_state()

        #check for distance to blocks
        if Field.in_deposit_boxes(self.robot.robotData.position, offset=0.05):
            return
        for id in self.field.field:
            if id == self.robot.robotData.targetBlock:
                continue
            if distance_segment_point_no_end(start, block_end, self.field.field[id][0]) < block_search_radius + 0.025:
                self._movement_stack((self.currentMovementState, self.target))
                self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw))
                self._movement_stack((RobotCommand.FORWARD, 1.5))
                if left_dist > right_dist:
                    self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw + 90))
                else:
                    self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw - 90))
                self._movement_stack((RobotCommand.FORWARD, 3))
                self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw))
                self._movement_stack((RobotCommand.FORWARD, 1.5))

                if left_dist > right_dist:
                    self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw - 90))
                else:
                    self._movement_stack((RobotCommand.TURN, self.robot.robotData.yaw + 90))
                self._movement_stack((RobotCommand.BACKWARD, 0.5))
                self.reset_movement_state()
                return


    def exit(self):
        self.currentLogicState = None
        self.currentMovementState = None
        self.movementQueue.clear()
        while not self.logicQueue.empty():
            self.logicQueue.get()
