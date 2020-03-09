import numpy as np
from tkinter import *

LEADER_COLOR = "blue"
D = 2
ROBOT_RADIUS = 25
LEADER_VEL = 5


class Visualizer:
    def __init__(self, follower_control, num_leader=1, num_follower=8):
        self.follower_control = follower_control
        self.nl = num_leader
        self.nf = num_follower
        self.window = Tk()
        self.window.attributes('-fullscreen', True)
        screen_size = self.window.maxsize()
        self.can = Canvas(
            self.window, width=screen_size[0], height=screen_size[1])
        self.can.pack()
        self.leader_pos = np.column_stack(
            ((np.arange(self.nl) + 1)*100, np.ones(self.nl) * 100))
        self.leader_id = [self.can.create_oval(
            self.leader_pos[i][0] - ROBOT_RADIUS,
            self.leader_pos[i][1]-ROBOT_RADIUS,
            self.leader_pos[i][0] + ROBOT_RADIUS,
            self.leader_pos[i][1]+ROBOT_RADIUS,
            fill=LEADER_COLOR, outline=LEADER_COLOR) for i in range(self.nl)]

        self.leader_vel = np.zeros((self.nl, D))

        self.window.bind("<KeyPress-Left>", self.left)
        self.window.bind("<KeyRelease-Left>", self.h_release)
        self.window.bind("<KeyPress-Right>", self.right)
        self.window.bind("<KeyRelease-Right>", self.h_release)
        self.window.bind("<KeyPress-Up>", self.up)
        self.window.bind("<KeyRelease-Up>", self.v_release)
        self.window.bind("<KeyPress-Down>", self.down)
        self.window.bind("<KeyRelease-Down>", self.v_release)

    def left(self, event):
        self.leader_vel[0][0] = -5

    def right(self, event):
        self.leader_vel[0][0] = 5

    def h_release(self, event):
        self.leader_vel[0][0] = 0

    def up(self, event):
        self.leader_vel[0][1] = -5

    def down(self, event):
        self.leader_vel[0][1] = 5

    def v_release(self, event):
        self.leader_vel[0][1] = 0

    def movement(self):
        for i in range(self.nl):
            self.can.move(
                self.leader_id[i], self.leader_vel[i][0], self.leader_vel[i][1])
        self.can.after(20, self.movement)

    def run(self):
        self.movement()
        self.window.mainloop()


class Robots:
    def __init__(self):
        self.robots = np.array([[0.0, 0],
                                [1, 2],
                                [2, 2],
                                [2, 1],
                                [0, 1],
                                [-1, -1],
                                [-1, -2],
                                [-2, -2],
                                [-2, -3],
                                [-1, -3]])
        self.followers = [1, 2, 3, 4, 5, 6, 7, 8, 9]
        self.leaders = [0]
        self.timestep = 0
        self.time_interval = 0.05
        self.d1 = 0.1
        self.d2 = 0
        self.ka = 1
        self.ks = 0.2
        self.show()

    def update(self, leaders_displacement=None):
        num_robots = self.robots.shape[0]
        num_followers = len(self.followers)
        num_leaders = len(self.leaders)
        if leaders_displacement is not None:
            for i in range(num_leaders):
                self.robots[self.leaders[i]] += leaders_displacement[i]
        dist_mat = np.linalg.norm(self.robots - np.expand_dims(self.robots, axis=1), axis=2)
        control_input = np.zeros((num_robots, 2))
        for f in self.followers:
            for i in range(num_robots):
                d = dist_mat[f, i] 
                if f != i:
                    control_input[f] += (-self.ka + self.ks / (d - self.d1) ** 2 / d) * (self.robots[f] - self.robots[i])
        for f in self.followers:
            self.robots[f] += control_input[f] * self.time_interval
        self.show()
    
    def show(self):
        for f in self.followers:
            plt.scatter(self.robots[f, 0], self.robots[f, 1], color='b')
        for l in self.leaders:
            plt.scatter(self.robots[l, 0], self.robots[l, 1], color='r')
        plt.show()


if __name__ == "__main__":
    vis = Visualizer(None)
    vis.run()
