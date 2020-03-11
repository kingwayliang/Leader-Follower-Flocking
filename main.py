import argparse
import numpy as np
from tkinter import *

from Params import *
from Visualization import *
from Controller import *
from Obstacle import *

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
    leader_pos = np.column_stack(
        ((np.arange(nl) + 1)*init_separation, np.ones(nl) * init_separation))
    follower_pos = np.column_stack(((np.arange(
        nf) + 1) * init_separation, screen_size[1] - np.ones(nf) * init_separation))

    obs1 = RectangularObstacle(500, 800, 100, 300)
    obs2 = RectangularObstacle(500, 800, 400, 600)
    obs3 = CircularObstacle(1000, 800, 50)
    obstacles = [obs1, obs2, obs3]

    control = PotentialController(
        np.row_stack((leader_pos, follower_pos)), nl, nf, obstacles=obstacles)

    vis = Visualizer(window, control, nl, nf, leader_pos,
                     follower_pos, obstacles=obstacles)
    vis.run()
