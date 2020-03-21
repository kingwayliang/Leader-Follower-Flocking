import numpy as np

from Params import *


def clip(x, low_bound, up_bound):
    return np.minimum(np.maximum(x, low_bound), up_bound)


def distanceMatrix(pos):
    return np.linalg.norm(pos - np.expand_dims(pos, axis=1), axis=2)
