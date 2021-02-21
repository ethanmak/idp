from .utils import *
import queue
from collections import deque
from .robot import Robot
from .command import *
from .field import Field, Color

class RobotStateMachine:
    """
    This class handles the logical flow of the program as well as the direct movement subroutines of the robot through a state machine
    """
    def __init__(self, robot: Robot, otherRobotData, field: Field = Field()):
        self.logicQueue = queue.Queue()  # queue of LogicCommands that are to be executed
        self.movementQueue = deque()  # queue of RobotCommands to be executed
        self.currentMovementState = None  # the current RobotCommand being executed
        self.currentLogicState = None  # the current LogicCommand being executed
        self.target = None  # the parameter of the RobotCommand, if applicable
        self.prev_state = None  # the previous state, typically used as a placeholder for a value in RobotData
        self.robot = robot  # the robot to interface with
        self.field = field  # robot field representation
        self.otherRobotData = otherRobotData  # other (non-target) RobotData
        self.prev_time = 0  # start time of RobotCommand
        
    def movement_queue(self, val: tuple) -> None:
        """
        Appends a RobotCommand in the form of (RobotCommand, Parameter if applicable) to end of queue

        :param val: Command to append to end of movementQueue
        :return: None
        """
        self.movementQueue.append(val)
        
    def _movement_stack(self, val: tuple) -> None:
        """
        Appends a RobotCommand in the form of (RobotCommand, Parameter if applicable) to start of queue

        :param val: Command to append to start of movementQueue
        :return: None
        """
        self.movementQueue.appendleft(val)

    def queue(self, val: tuple) -> None:
        """
        Appends a LogicCommand in the form of (LogicCommand, Parameter if applicable) to end of queue

        :param val: Command to append to end of logicQueue
        :return: None
        """
        self.logicQueue.put(val)

    def timed_out(self):
        """
        Return whether last movement command timed out or not

        :return: True if timed out, False otherwise
        """
        timed = self.robot.get_simulation_time() - self.prev_time > 12
        if timed:
            print('Timed Out')
        return timed

    def reset_movement_state(self) -> None:
        """
        Resets currentMovementState to dormant value (None)

        :return: None
        """
        self.currentMovementState = None

    def movement_command_empty(self):
        """
        Returns whether all movement commands have been processed

        :return: True if totally processed, False otherwise
        """
        return len(self.movementQueue) <= 0 and self.currentMovementState is None

    def reset(self) -> None:
        """
        Clears queue of all commands and resets current states

        :return: None
        """
        self.currentLogicState = None
        self.currentMovementState = None
        self.movementQueue.clear()
        while not self.logicQueue.empty():
            self.logicQueue.get()

    def update_movement(self) -> None:
        """
        Updates robot devices, motor velocities, and field representation based on currentMovementCommand, and queues a new one if current one has finished
        Refer to command.RobotCommand for information on what each command does

        :return: None
        """
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
            """
            In order to detect blocks, this robot employs a radial sweep line algorithm and checks the difference between the distanced detected and the ideal distance to the walls.
            The distance to the wall is calculated using a trick with rays and line segment to avoid using another distance sensor.
            
            If the difference is large enough, a block position based on the heading and distance is calculated and logged in the field representation.
            """
            robot_dist = 0.01  # distance of robot from GPS sensor to distance sensor
            dist = self.robot.get_distance()
            rotation = self.robot.robotData.yaw
            wall_dist = Field.distance_to_wall(self.robot.robotData.position, rotation) - robot_dist  # accounting for robot depth
            wall_dist = clamp(wall_dist, 0, 1.5)  # accounting for the max range of the distance sensor, 150cm
            if wall_dist - dist > 0.02 and 0.06 < dist < 1.5 and dist < wall_dist:
                dist += 0.025 + robot_dist  # accounts for depth of block when calculating position
                rotation += 0.5  # accounting for the fact that the robot detects the block likely on the first edge, so we must add 0.5 degrees to find the center of the block
                angle = np.radians(rotation)
                point = np.array([self.robot.robotData.position[0] + dist * np.cos(angle),
                                  self.robot.robotData.position[1] + dist * np.sin(angle)])  # center of block
                if not self.field.contains_point(point, threshold=0.05 * 1.5) \
                        and Field.in_bounds(point) \
                        and np.linalg.norm(point - self.otherRobotData.position) > 0.2\
                        and not Field.in_deposit_boxes(point):
                    """
                    Whether to add the block is determined by the following factors:
                    If the block detected has already been detected
                    If the block detected is in bounds
                    If the block detected is far enough away from the other robot (could just be the other robot being detected)
                    If the block is not in the depost boxes
                    """
                    self.field.add_block(point, use_field=self.target)
            if self.robot.robotData.yaw < self.prev_state - 0.05 and self.prev_state - self.robot.robotData.yaw < 1:
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
    
    def update_logic(self):
        """
        Queues RobotCommands into movementQueue when current LogicCommand has finished its task
        Refer to command.LogicCommand for information about what each command does

        :return: None
        """
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
                # remove blocks that have colors that are misdetected to account for possible misdetections in sweeping
                self.field.remove_block(self.robot.robotData.targetBlock)
            if color == self.robot.robotData.color:
                # if the block is the right color, then capture
                self.queue((LogicCommand.CAPTURE,))
                self.queue((LogicCommand.TRAVEL_BACK,))
                self.queue((LogicCommand.DEPOSIT,))
            else:
                # if the block is the wrong color, then sweep again
                self.movement_queue((RobotCommand.BACKWARD, 1.2))
                self.queue((LogicCommand.SEARCH,))
                self.robot.robotData.targetBlock = -1
        elif state == LogicCommand.DELAY:
            self.movement_queue((RobotCommand.DELAY, target))
    
    def check_failsafes(self, give_way: bool = False) -> None:
        """
        Dynamically checks for possible issues ie. collisions and pauses current RobotCommand to enact evasive maneuvers

        Currently implemented failsafes:
        Impending collision with block
        Impending collision with other robot
        Stalemates (where the robots stop to avoid collision but remain still waiting for the other to pass)

        :param give_way: Whether to move out of path if in stalemate (Default is False)
        :return: None
        """

        """
        In order to detect issues, the robot employs virtual sensors, sensors that exist mathematically rather than physically.
        More specifically, an imaginary line segment is drawn from the front of the robot (start) to an endpoint (block_end for block detection and robot_end for robot detection).
        When a block or robot is too close to their respective line segment, the current RobotCommand is paused and evasive maneuvers are stacked to be run. 
        """
        block_search_radius = 0.071
        robot_search_radius = 0.051 * 1.5

        dir = degree_to_vector(self.robot.robotData.yaw)
        start = self.robot.robotData.position + 0.064 * dir
        robot_end = start + dir * robot_search_radius * 1.2
        block_end = start + dir * 0.08

        # left and right distances of robot to wall
        left_dist = Field.distance_to_wall(self.robot.robotData.position, self.robot.robotData.yaw - 90)
        right_dist = Field.distance_to_wall(self.robot.robotData.position, self.robot.robotData.yaw + 90)

        # evasive maneuvers should only be taken when the robot is traveling or in a paused state in which it is waiting for the right of way
        if self.currentMovementState != RobotCommand.POINT and self.currentMovementState != RobotCommand.PAUSE:
            return

        #check for frontal distance to other robot
        robot_dist = distance_segment_point(start, robot_end, self.otherRobotData.position)
        if robot_dist < robot_search_radius + robot_search_radius * 1.4:
            if abs(self.otherRobotData.yaw - self.robot.robotData.yaw - 180) < 60 and give_way:
                # a possible stalemate is detected where the other robot is going for a head on collision
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
                # if not head on then pause the robot to wait for other robot to pass
                self._movement_stack((self.currentMovementState, self.target))
                self._movement_stack((RobotCommand.PAUSE,))
                self.reset_movement_state()
        elif self.currentMovementState == RobotCommand.PAUSE:
            self.reset_movement_state()

        #check for distance to blocks
        if Field.in_deposit_boxes(self.robot.robotData.position, offset=0.05):
            # if robot is in deposit boxes do not run block detection
            return

        for id in self.field.field:
            if id == self.robot.robotData.targetBlock:
                continue
            if distance_segment_point_no_end(start, block_end, self.field.field[id][0]) < block_search_radius + 0.025:
                # if found a possible block collision in front
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
