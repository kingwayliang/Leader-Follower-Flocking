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


if __name__ == "__main__":
    vis = Visualizer(None)
    vis.run()
