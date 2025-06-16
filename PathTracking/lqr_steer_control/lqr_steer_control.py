import math
import pathlib
import sys
import scipy.linalg as la
import matplotlib.pyplot as plt
import numpy as np
from utils.angle import angle_mod

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from PathPlanning.CubicSpline import cubic_spline_planner


Kp = 1.0 # Speed Proportional Gain

# LQR parameter
Q = np.eye(4)
R = np.eye(1)

print(Q, '\n', R)