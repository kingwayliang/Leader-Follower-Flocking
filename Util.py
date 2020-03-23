import numpy as np

from Params import *


def clip(x, low_bound, up_bound):
    return np.minimum(np.maximum(x, low_bound), up_bound)


def distanceMatrix(pos):
    return np.linalg.norm(pos - np.expand_dims(pos, axis=1), axis=2)


def computeCircularPos(n, radius, x_center, y_center):
    angles = np.arange(n) * 2 * np.pi / n
    xpos = np.cos(angles) * radius + x_center
    ypos = np.sin(angles) * radius + y_center
    return np.column_stack((xpos, ypos))
