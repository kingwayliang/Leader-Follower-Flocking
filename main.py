import argparse
import numpy as npd
from tkinter import *

from Params import *
from Visualization import *
from Controller import *
from Obstacle import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-nl", type=int, default=2)
    parser.add_argument("-nf", type=int, default=8)
    parser.add_argument("-ob", type=bool, default=True)
    parser.add_argument("-ct", type=str, default="flock")
    args = parser.parse_args()

    nl = args.nl
    nf = args.nf

    window = Tk()
    window.attributes('-fullscreen', True)
    screen_size = window.maxsize()

    init_separation = ROBOT_RADIUS * 5

    robot_pos = computeCircularPos(
        nl + nf, min(300, (nl+nf) * 30), screen_size[0] / 2, screen_size[1] / 2)

    leader_pos = robot_pos[:nl]
    follower_pos = robot_pos[nl:]

    obs1 = CircularObstacle(200, 250, 50)
    obs2 = CircularObstacle(200, 400, 50)
    obs3 = CircularObstacle(200, 540, 50)
    obs4 = RectangularObstacle(1200, 1400, 200, 300)
    obs5 = RectangularObstacle(1200, 1400, 370, 450)
    obs6 = RectangularObstacle(1200, 1400, 550, 650)
    if args.ob:
        obstacles = [obs1, obs2, obs3, obs4, obs5, obs6]
    else:
        obstacles = None

    if args.ct == "flock":
        control = FlockCenterController(robot_pos, nl, nf, obstacles=obstacles)
    elif args.ct == "connect":
        control = ConnectivityMaintenanceController(
            robot_pos, nl, nf, obstacles)
    elif args.ct == "cubic":
        control = CubicController(robot_pos, nl, nf, obstacles)
    else:
        control = PotentialController(robot_pos, nl, nf, obstacles=obstacles)

    vis = Visualizer(window, control, nl, nf, leader_pos,
                     follower_pos, obstacles=obstacles)
    vis.run()
