import numpy as np
from tkinter import *

from Controller import *
from Params import *
from Util import *


class Visualizer:
    def __init__(self, window, follower_control, num_leader, num_follower, leader_pos, follower_pos, obstacles=None):

        # set up window and canvas
        self.window = window
        screen_size = self.window.maxsize()
        self.can = Canvas(
            self.window, width=screen_size[0], height=screen_size[1])
        self.can.pack()

        # save self variables
        self.follower_control = follower_control
        self.nl = num_leader
        self.nf = num_follower
        self.obstacles = obstacles

        if obstacles:
            for obs in obstacles:
                if obs.obs_type == CIRCLE:
                    self.can.create_oval(obs.x-obs.radius, obs.y-obs.radius, obs.x +
                                         obs.radius, obs.y+obs.radius, fill=obs.color, outline=obs.color)
                if obs.obs_type == RECTANGLE:
                    self.can.create_rectangle(
                        obs.x1, obs.y1, obs.x2, obs.y2, fill=obs.color, outline=obs.color)

        # ids for interaction in canvas
        self.leader_id = [self.can.create_oval(
            leader_pos[i][0] - ROBOT_RADIUS,
            leader_pos[i][1]-ROBOT_RADIUS,
            leader_pos[i][0] + ROBOT_RADIUS,
            leader_pos[i][1]+ROBOT_RADIUS,
            fill=LEADER_COLOR, outline=LEADER_COLOR) for i in range(self.nl)]

        self.follower_id = [self.can.create_oval(
            follower_pos[i][0] - ROBOT_RADIUS,
            follower_pos[i][1]-ROBOT_RADIUS,
            follower_pos[i][0] + ROBOT_RADIUS,
            follower_pos[i][1]+ROBOT_RADIUS,
            fill=FOLLOWER_COLOR, outline=FOLLOWER_COLOR) for i in range(self.nf)]

        # for movement update
        self.follower_displ = np.zeros((self.nf, D))
        self.leader_displ = np.zeros((self.nl, D))

        # bind keys for leaders
        if self.nl > 0:
            self.window.bind("<KeyPress-Left>", self.left)
            self.window.bind("<KeyRelease-Left>", self.h_release)
            self.window.bind("<KeyPress-Right>", self.right)
            self.window.bind("<KeyRelease-Right>", self.h_release)
            self.window.bind("<KeyPress-Up>", self.up)
            self.window.bind("<KeyRelease-Up>", self.v_release)
            self.window.bind("<KeyPress-Down>", self.down)
            self.window.bind("<KeyRelease-Down>", self.v_release)
        if self.nl > 1:
            self.window.bind("<KeyPress-a>", self.key_a)
            self.window.bind("<KeyRelease-a>", self.ad_release)
            self.window.bind("<KeyPress-d>", self.key_d)
            self.window.bind("<KeyRelease-d>", self.ad_release)
            self.window.bind("<KeyPress-w>", self.key_w)
            self.window.bind("<KeyRelease-w>", self.ws_release)
            self.window.bind("<KeyPress-s>", self.key_s)
            self.window.bind("<KeyRelease-s>", self.ws_release)

    def left(self, event):
        self.leader_displ[0][0] = -5

    def right(self, event):
        self.leader_displ[0][0] = 5

    def h_release(self, event):
        self.leader_displ[0][0] = 0

    def up(self, event):
        self.leader_displ[0][1] = -5

    def down(self, event):
        self.leader_displ[0][1] = 5

    def v_release(self, event):
        self.leader_displ[0][1] = 0

    def key_a(self, event):
        self.leader_displ[1][0] = -5

    def key_d(self, event):
        self.leader_displ[1][0] = 5

    def ad_release(self, event):
        self.leader_displ[1][0] = 0

    def key_w(self, event):
        self.leader_displ[1][1] = -5

    def key_s(self, event):
        self.leader_displ[1][1] = 5

    def ws_release(self, event):
        self.leader_displ[1][1] = 0

    def movement(self):
        self.follower_displ = self.follower_control.update(self.leader_displ)
        for i in range(self.nf):
            self.can.move(
                self.follower_id[i], self.follower_displ[i][0], self.follower_displ[i][1])
        for i in range(self.nl):
            self.can.move(
                self.leader_id[i], self.leader_displ[i][0], self.leader_displ[i][1])
        self.can.after(CANVAS_DELAY, self.movement)

    def run(self):
        self.movement()
        self.window.mainloop()


if __name__ == "__main__":
    vis = Visualizer(Tk(), None, 3, 32)
    vis.run()
