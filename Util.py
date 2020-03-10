import numpy as np


def clip(x, low_bound, up_bound):
    return np.minimum(np.maximum(x, low_bound), up_bound)
