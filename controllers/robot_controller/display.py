import pygame
import numpy as np

from robot_controller import RobotData
from robot_controller.field import Field


class FieldDisplay:
    """
    This class handles all of the GUI rendering needed for the field representation
    """

    def __init__(self, resolution: int = 800, title: str = 'Field Display'):
        """
        Initializer
        Starts pygame and sets up internal variables for field representation
        :param resolution: Resolution of window
        :param title: Title of window
        """
        pygame.init()
        self.resolution = resolution
        self.screen = pygame.display.set_mode([resolution, resolution])
        pygame.display.set_caption(title)
        self.botSize = resolution * 0.102 / 2.4
        self.blockSize = resolution * 0.05 / 2.4

    def _draw_field(self, field: dict) -> None:
        """
        Draws blocks from field representation onto screen

        :param field: Field representation to draw
        :return: None
        """
        for i in field:
            pos = self._map_to_pixel(field[i][0])
            color = field[i][-1]
            pygame.draw.rect(self.screen, color.value, [pos[0] - self.blockSize/2, pos[1] - self.blockSize/2, self.blockSize, self.blockSize])

    def _draw_target(self, field: dict, robotData: RobotData, color: tuple, thickness: int = 2) -> None:
        """
        Draws a colored circle around the target block of robotData

        :param field: Dictionary containing blocks of field representation
        :param robotData: RobotData of target robot
        :param color: Color of circle
        :param thickness: Thickness of circle
        :return: None
        """
        if robotData.targetBlock >= 0:
            pygame.draw.circle(self.screen, color, self._map_to_pixel(field[robotData.targetBlock][0]), self.blockSize / 2 * 1.41 + thickness, thickness)


    def _draw_robot(self, position, angle, color) -> None:
        """
        Draws the robot as a circular body with line for heading on screen

        :param position: Position of robot
        :param angle: Heading of robot
        :param color: Color of robot
        :return: None
        """
        angle = np.radians(angle)
        center = self._map_to_pixel(position)
        pygame.draw.circle(self.screen, color, center, self.botSize)
        p2_line = (center[0] - self.botSize * np.cos(angle), center[1] - self.botSize * np.sin(angle))
        pygame.draw.line(self.screen, (0, 0, 0), center, p2_line)

    def _draw_deposit(self) -> None:
        """
        Draws deposit boxes on screen

        :return: None
        """
        pygame.draw.rect(self.screen, (84, 84, 255),
                         [0, self.resolution * 2 / 2.4, self.resolution * 0.4 / 2.4, self.resolution * 0.4 / 2.4])
        pygame.draw.rect(self.screen, (255, 84, 84),
                         [0, 0, self.resolution * 0.4 / 2.4, self.resolution * 0.4 / 2.4])

    def draw(self, blueRobotData: RobotData, redRobotData: RobotData, field: Field, progress: bool = True) -> None:
        """
        Draws robots and field representation onto screen

        :param blueRobotData: RobotData of blue robot
        :param redRobotData: RobotData of red robot
        :param field: Field representation
        :param progress: Whether to handle pygame events (True by default, and needed to tell Windows that GUI is responding)
        :return: None
        """
        if progress:
            pygame.event.get()
        self.screen.fill((255, 255, 255))
        self._draw_deposit()
        if blueRobotData is not None:
            self._draw_robot(blueRobotData.position, blueRobotData.yaw, (0, 0, 255))
            self._draw_target(field.field, blueRobotData, (110, 190, 255))
        if redRobotData is not None:
            self._draw_robot(redRobotData.position, redRobotData.yaw, (255, 0, 0))
            self._draw_target(field.field, redRobotData, (255, 92, 92))
        if field is not None:
            self._draw_field(field.field)
        pygame.display.flip()

    def exit(self) -> None:
        """
        Exits pygame and closes screen

        :return: None
        """
        pygame.quit()

    def _map_to_pixel(self, coord: np.ndarray) -> tuple:
        """
        Converts simulation coordinates to screen coordinates

        :param coord: Game coordinates
        :return: Converted screen coordinates
        """
        return self.resolution - (coord[0] + 1.2) / 2.4 * self.resolution, self.resolution - (coord[1] + 1.2) / 2.4 * self.resolution
