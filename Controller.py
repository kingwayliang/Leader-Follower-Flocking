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
            self.obs_offset_calc = ObstacleOffsetCalculator(obstacles)
        else:
            self.obs_offset_calc = None

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
                d = dist_mat[f, i]
                if f != i and d < SENSING_RANGE:
                    fi = f - self.nl
                    control_input[fi] += (-self.ka + self.ks / (d - self.d1)
                                          ** 2 / d) * (self.robot_pos[f] - self.robot_pos[i])
        self.robot_pos[self.nl:, :] += control_input * self.time_interval
        self.robot_pos[:self.nl, :] += leader_displ
        return control_input * self.time_interval


class FlockCenterController(Controller):
    def __init__(self, robot_pos, num_leader, num_follower, obstacles=None):
        super().__init__(robot_pos, num_leader, num_follower, obstacles)
        self.centers = robot_pos.copy()
        self.num_robots = num_leader + num_follower
        self.time_interval = 0.004
        self.d1 = 0
        self.d2 = ROBOT_RADIUS * 30
        self.ka = 1.5
        self.ks = 600000
        self.kc = 0.1

    def update(self, leader_vel=None):
        if leader_vel is not None:
            self.robot_pos[:self.nl, :] += leader_vel
        dist_mat = np.linalg.norm(
            self.robot_pos - np.expand_dims(self.robot_pos, axis=1), axis=2)
        # update flock centers
        delta_centers = np.zeros((self.num_robots, 2))
        for i in range(self.num_robots):
            delta_centers[i] += self.robot_pos[i] - self.centers[i]
            for j in range(self.num_robots):
                if i != j and dist_mat[i, j] < self.d2:
                    delta_centers[i] += self.centers[j] - self.centers[i]
        for i in range(self.num_robots):
            self.centers[i] += delta_centers[i] * self.kc
        # calculate control input
        obs_offset = None
        if self.obs_offset_calc:
            obs_offset = self.obs_offset_calc.offset_from_obstacles(
                self.robot_pos)
            obs_dist = np.linalg.norm(obs_offset, axis=2)
        control_input = np.zeros((self.nf, 2))
        for f in range(self.nl, self.num_robots):
            # attraction to flock center
            control_input[f - self.nl] += self.ka * np.linalg.norm(
                (self.centers[f] - self.robot_pos[f])) * (self.centers[f] - self.robot_pos[f])
            # sparation
            if obs_offset is not None:
                for o in range(obs_offset.shape[1]):
                    d = obs_dist[f, o]
                    sparate_force = self.ks / (d - self.d1) ** 2 / d
                    control_input[f - self.nl] += sparate_force * \
                        obs_offset[f, o]
            for i in range(self.num_robots):
                d = dist_mat[f, i]
                if i != f:
                    sparate_force = self.ks / (d - self.d1) ** 2 / d
                    control_input[f - self.nl] += sparate_force * \
                        (self.robot_pos[f] - self.robot_pos[i])
        self.robot_pos[self.nl:, :] += control_input * self.time_interval
        return control_input * self.time_interval
