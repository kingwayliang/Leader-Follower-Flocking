import argparse
import numpy as npd
from tkinter import *

from Params import *
from Visualization import *
from Controller import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-nl", type=int, default=2)
    parser.add_argument("-nf", type=int, default=8)
    args = parser.parse_args()

    nl = args.nl
    nf = args.nf

    window = Tk()
    window.attributes('-fullscreen', True)
    screen_size = window.maxsize()

    init_separation = ROBOT_RADIUS * 5
#    leader_pos = np.column_stack(
#        ((np.arange(nl) + 1)*init_separation, np.ones(nl) * init_separation))
#    follower_pos = np.column_stack(((np.arange(
#        nf) + 1) * init_separation, screen_size[1] - np.ones(nf) * init_separation))
    robot_pos = np.array([[0.0, 0],
                [1, 2],
                [2, 2],
                [2, 1],
                [0, 1],
                [-1, -1],
                [-1, -2],
                [-2, -2],
                [-2, -3],
                [-1, -3]]) * init_separation + [screen_size[0] / 2, screen_size[1] / 2]
    leader_pos = robot_pos[:2]
    follower_pos = robot_pos[2:]
    
    control = SmallRangeController(
        np.row_stack((leader_pos, follower_pos)), nl, nf)

    vis = Visualizer(window, control, nl, nf, leader_pos, follower_pos)
    vis.run()
