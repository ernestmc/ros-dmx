import math
import pygame

class PanGauge(object):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.angle = 0
        self.colors = {
            'border': pygame.Color(200, 0, 0),
            'marker': pygame.Color(200, 200, 200),
            'text': pygame.Color(200, 0, 0),
        }
        pygame.font.init()
        self.font = pygame.font.SysFont('Courier new bold', 20)

    def set_pan(self, pan):
        self.angle = pan

    def draw(self, surface, x=0, y=0):
        # type: (pygame.Surface, int, int) -> None
        self.draw_border(surface, x, y)
        self.draw_text(surface, x + 5, y + 5, 'Pan: %d' % math.ceil(math.degrees(self.angle)))
        color = self.colors['marker']
        start = [x + self.width / 2, y + self.height]
        radius = self.width / 2
        end = [start[0] - math.cos(self.angle + math.pi/2) * radius, start[1] - math.sin(self.angle + math.pi/2) * radius]
        pygame.draw.line(surface, color, start, end, 2)

    def draw_border(self, surface, x, y):
        # type: (pygame.Surface, int, int) -> None
        color = self.colors['border']
        width = 2
        pygame.draw.rect(surface, color, (x, y, self.width, self.height), width)

    def draw_text(self, surface, x, y, text):
        # type: (pygame.Surface, int, int, string) -> None
        color = self.colors['text']
        textsurface = self.font.render(text, False, color)
        surface.blit(textsurface, (x, y))

class TiltGauge(object):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.angle = 0
        self.colors = {
            'border': pygame.Color(200, 0, 0),
            'marker': pygame.Color(200, 200, 200),
            'text': pygame.Color(200, 0, 0),
        }
        pygame.font.init()
        self.font = pygame.font.SysFont('Courier new bold', 20)

    def set_tilt(self, tilt):
        self.angle = tilt

    def draw(self, surface, x=0, y=0):
        # type: (pygame.Surface, int, int) -> None
        self.draw_border(surface, x, y)
        self.draw_text(surface, x + 5, y + 5, 'Tilt: %d' % math.ceil(math.degrees(self.angle)))
        color = self.colors['marker']
        start = [x + self.width, y + self.height / 2]
        radius = self.height / 2
        end = [start[0] + math.cos(self.angle + math.pi) * radius, start[1] + math.sin(self.angle + math.pi) * radius]
        pygame.draw.line(surface, color, start, end, 2)

    def draw_border(self, surface, x, y):
        # type: (pygame.Surface, int, int) -> None
        color = self.colors['border']
        width = 2
        pygame.draw.rect(surface, color, (x, y, self.width, self.height), width)

    def draw_text(self, surface, x, y, text):
        # type: (pygame.Surface, int, int, string) -> None
        color = self.colors['text']
        textsurface = self.font.render(text, False, color)
        surface.blit(textsurface, (x, y))
