import numpy as np

from Params import *
from Obstacle import *


class Controller:
    '''
    Base class for multi-agent controller, to be used with Visualizer
    '''

    def __init__(self, robot_pos, num_leader, num_follower, obstacles=None):
        '''
        robot_pos: np array of size (num_robot, D), leader first, then follower
        '''
        self.robot_pos = robot_pos
        self.nl = num_leader
        self.nf = num_follower
        if obstacles:
            self.obs_dist_calc = ObstacleDistanceCalculator(obstacles)
        else:
            self.obs_dist_calc = None

    def update(self, leader_displ):
        '''
        This function should directly update self.robot_pos and return displacement of followers
        '''
        raise NotImplementedError("Need to implement update in derived class")


class DummyController(Controller):
    def update(self, leader_displ):
        follower_disp = (self.robot_pos[0, :] -
                         self.robot_pos[self.nl:, :]) * 0.05
        self.robot_pos += np.row_stack((leader_displ, follower_disp))
        return follower_disp


class PotentialController(Controller):
    def __init__(self, robot_pos, num_leader, num_follower, obstacles=None):
        super().__init__(robot_pos, num_leader, num_follower, obstacles)
        self.num_robots = num_leader + num_follower
        self.timestep = 0
        self.time_interval = 0.005
        self.d1 = 0
        self.d2 = ROBOT_RADIUS * 2
        self.ka = 1
        self.ks = 600000
        print(robot_pos)

    def update(self, leader_displ):
        dist_mat = np.linalg.norm(
            self.robot_pos - np.expand_dims(self.robot_pos, axis=1), axis=2)
        control_input = np.zeros((self.nf, 2))
        for f in range(self.nl, self.num_robots):
            for i in range(self.num_robots):
                if f != i:
                    d = dist_mat[f, i]
                    fi = f - self.nl
                    control_input[fi] += (-self.ka + self.ks / (d - self.d1)
                                          ** 2 / d) * (self.robot_pos[f] - self.robot_pos[i])
        self.robot_pos[self.nl:, :] += control_input * self.time_interval
        self.robot_pos[:self.nl, :] += leader_displ
        return control_input * self.time_interval
