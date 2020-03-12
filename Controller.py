import numpy as np

from Params import *


class Controller:
    '''
    Base class for multi-agent controller, to be used with Visualizer
    '''

    def __init__(self, robot_pos, num_leader, num_follower, obstacles=None):
        '''
        robot_pos: np array of size (num_robot, D), leader first, then follower
        robot_vel: same size as robot_pos
        '''
        self.robot_pos = robot_pos
        self.nl = num_leader
        self.nf = num_follower
        self.obstacles = obstacles

    def update(self, leader_vel):
        '''
        This function should directly update self.robot_pos and return displacement of followers
        '''
        raise NotImplementedError("Need to implement update in derived class")


class DummyController(Controller):
    def update(self, leader_vel):
        follower_disp = (self.robot_pos[0, :] -
                         self.robot_pos[self.nl:, :]) * 0.05
        self.robot_pos += np.row_stack((leader_vel, follower_disp))
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

    def update(self, leader_vel=None):
        if leader_vel is not None:
            self.robot_pos[:self.nl, :] += leader_vel
        dist_mat = np.linalg.norm(
            self.robot_pos - np.expand_dims(self.robot_pos, axis=1), axis=2)
        control_input = np.zeros((self.nf, 2))
        for f in range(self.nl, self.num_robots):
            for i in range(self.num_robots):
                d = dist_mat[f, i]
                if f != i:  
                    fi = f - self.nl
                    control_input[fi] += (-self.ka + self.ks / (d - self.d1)
                                          ** 2 / d) * (self.robot_pos[f] - self.robot_pos[i])
        self.robot_pos[self.nl:, :] += control_input * self.time_interval
        return control_input * self.time_interval
 
class SmallRangeController(Controller):
    def __init__(self, robot_pos, num_leader, num_follower, obstacles=None):
        super().__init__(robot_pos, num_leader, num_follower, obstacles)
        self.centers = robot_pos.copy()
        self.num_robots = num_leader + num_follower
        self.timestep = 0
        self.time_interval = 0.01
        self.d1 = 0
        self.d2 = ROBOT_RADIUS * 10
        self.d3 = ROBOT_RADIUS * 4
        self.ka = 40
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
        control_input = np.zeros((self.nf, 2))
        for f in range(self.nl, self.num_robots):
            # attraction to flock center
            control_input[f - self.nl] += self.ka * (self.centers[f] - self.robot_pos[f])
            # sparation
            for i in range(self.num_robots):
                d = dist_mat[f, i]
                if i != f:
                    sparate_force = self.ks / (d - self.d1) ** 2 / d
                    control_input[f - self.nl] += sparate_force * (self.robot_pos[f] - self.robot_pos[i])
        self.robot_pos[self.nl:, :] += control_input * self.time_interval
        return control_input * self.time_interval
 
    