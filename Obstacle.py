from Params import *


class Obstacle:
    '''
    Base class for obstacles
    '''

    def __init__(self, obs_type):
        self.obs_type = obs_type


class CircularObstacle(Obstacle):
    def __init__(self, x, y, radius, color="black"):
        '''
        center and radius
        '''
        super().__init__(CIRCLE)
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color


class RectangularObstacle(Obstacle):
    def __init__(self, x1, x2, y1, y2, color="black"):
        '''
        x y of 4 corners
        '''
        super().__init__(RECTANGLE)
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.color = color
