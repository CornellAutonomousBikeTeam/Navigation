import math
import numpy as np


# Takes an angle in radians, r, and returns an equivalent angle that is within PI of center
def align_radians(r, center=0):
    return r - math.floor((r - center) / (2 * np.pi) + 0.5) * 2 * np.pi


# Distance (a - b) in radians. Will always be within +/- PI.
def dist_radians(a, b):
    return a - align_radians(b, center=a)