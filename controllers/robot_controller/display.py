import pygame
import numpy as np

class FieldDisplay:
    def __init__(self,resolution=800, title='Field Display'):
        pygame.init()
        self.resolution = resolution
        self.screen = pygame.display.set_mode([resolution, resolution])
        pygame.display.set_caption(title)
        self.blueDeposit = None
        self.redDeposit = None
        self.botSize = resolution * 0.12 / 2.4
        self.blockSize = resolution * 0.05 / 2.4

    def _draw_field(self, field):
        for i in field:
            pos = self.map_to_pixel(field[i][0])
            color = field[i][-1]
            pygame.draw.rect(self.screen, color.value, [pos[0] - self.blockSize/2, pos[1] + self.blockSize/2, self.blockSize, self.blockSize])

    def _set_robot_position(self, position, angle, color):
        angle = np.radians(angle)
        center = self.map_to_pixel(position)
        pygame.draw.circle(self.screen, color, center, self.botSize)
        p2_line = (center[0] - self.botSize * np.cos(angle), center[1] - self.botSize * np.sin(angle))
        pygame.draw.line(self.screen, (0, 0, 0), center, p2_line)

    def _draw_deposit(self):
        pygame.draw.rect(self.screen, (84, 84, 255),
                         [0, self.resolution * 2 / 2.4, self.resolution * 0.4 / 2.4, self.resolution * 0.4 / 2.4])
        pygame.draw.rect(self.screen, (255, 84, 84),
                         [0, 0, self.resolution * 0.4 / 2.4, self.resolution * 0.4 / 2.4])

    def draw(self, blueRobotData, redRobotData, field):
        pygame.event.get()
        self.screen.fill((255, 255, 255))
        self._draw_deposit()
        if blueRobotData is not None:
            self._set_robot_position(blueRobotData.position, blueRobotData.yaw, (0, 0, 255))
        if redRobotData is not None:
            self._set_robot_position(redRobotData.position, redRobotData.yaw, (255, 0, 0))
        if field is not None:
            self._draw_field(field.field)
        pygame.display.flip()

    def exit(self):
        pygame.quit()

    def map_to_pixel(self, coord):
        return self.resolution - (coord[0] + 1.2) / 2.4 * self.resolution, self.resolution - (coord[1] + 1.2) / 2.4 * self.resolution
