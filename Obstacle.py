import numpy as np

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


class ObstacleDistanceCalculator:
    def __init__(self, obstacles):
        self.circle_centers = np.zeros((1, D, 0))
        self.rect_centers = np.zeros((1, D, 0))
        self.rect_halfwh = np.zeros(((1, D, 0)))
        radii = []
        for obs in obstacles:
            if obs.obs_type == CIRCLE:
                self.circle_centers = np.append(self.circle_centers, np.array(
                    [obs.x, obs.y]).reshape((1, D, 1)), axis=2)
                radii.append(obs.radius)
            elif obs.obs_type == RECTANGLE:
                center = np.array(
                    [obs.x1+obs.x2, obs.y1+obs.y2]).reshape((1, D, 1)) / 2
                self.rect_centers = np.append(
                    self.rect_centers, center, axis=2)
                halfwh = np.abs(
                    np.array([obs.x1-obs.x2, obs.y1-obs.y2])).reshape((1, D, 1)) / 2
                self.rect_halfwh = np.append(self.rect_halfwh, halfwh, axis=2)
        self.circle_radii = np.expand_dims(np.array(radii), axis=0)

    def dist_to_cir(self, robot_pos):
        '''
        robot_pos: pos of robot for distances, should be followers
        returns dist of size (num_robot, num_circles)
        radius already accounted for
        '''
        diff = np.expand_dims(robot_pos, 2) - self.circle_centers
        return np.linalg.norm(diff, axis=1) - self.circle_radii

    def dist_to_rect(self, robot_pos):
        '''
        robot_pos: pos of robot for distances, should be followers
        returns dist of size (num_robot, num_circles)
        '''
        diff = np.abs(np.expand_dims(robot_pos, 2) -
                      self.rect_centers) - self.rect_halfwh
        diff = np.maximum(diff, 0)
        return np.linalg.norm(diff, axis=1)
