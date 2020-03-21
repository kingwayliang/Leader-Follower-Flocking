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


class ObstacleOffsetCalculator:
    '''
    Utility class for calculating position vector pointing from
    closest point on obstacles to each robot. Init with list of
    obstacles, call offset_from_obstacles with related robot
    positions to get a (num_robot, num_obstacle, D) matrix
    '''

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

    def offset_from_cir(self, robot_pos):
        '''
        robot_pos: pos of robot for distances, should be followers
        returns vector from closest point on circle to robot
        return size (num_robot, num_circle, D)
        '''
        diff = np.expand_dims(robot_pos, 2) - self.circle_centers
        dist_from_center = np.linalg.norm(diff, axis=1)
        dist_from_closest = dist_from_center - self.circle_radii
        scaling = dist_from_closest / dist_from_center
        diff = np.swapaxes(diff, 1, 2)
        return diff * np.expand_dims(scaling, 2)

    def offset_from_rect(self, robot_pos):
        '''
        robot_pos: pos of robot for distances, should be followers
        returns vector from closest point on rectangle to robot
        return size (num_robot, num_rect, D)
        '''
        diff_from_center = np.expand_dims(robot_pos, 2) - self.rect_centers
        sign = np.sign(diff_from_center)
        diff = np.abs(diff_from_center) - self.rect_halfwh
        diff = np.maximum(diff, 0) * sign
        return np.swapaxes(diff, 1, 2)

    def offset_from_obstacles(self, robot_pos):
        '''
        robot_pos: pos of robot for distances, should be followers
        returns vector from closest point on obstacle to robot
        return size (num_robot, num_obstacle, D)
        '''
        ofs_cir = self.offset_from_cir(robot_pos)
        ofs_rec = self.offset_from_rect(robot_pos)
        return np.concatenate([ofs_cir, ofs_rec], axis=1)
