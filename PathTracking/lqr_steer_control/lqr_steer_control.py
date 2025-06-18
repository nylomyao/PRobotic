import math
import pathlib
import sys
import scipy.linalg as la
import matplotlib.pyplot as plt
import numpy as np

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
from utils.angle import angle_mod
from PathPlanning.CubicSpline import cubic_spline_planner


Kp = 1.0 # Speed Proportional Gain

# LQR parameter
Q = np.eye(4)
R = np.eye(1)


# parameters
dt = 0.1 # [s] time tick
L = 0.5 # [m] wheelbase of the vehicle
max_steer = np.deg2rad(45.0) # [rad] maximum steering angle

show_animation = True
# show_animation = False

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

# 更新车辆状态
def update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    if delta <= -max_steer:
        delta = -max_steer
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.x = state.v + a * dt

    return state

def pid_control(target, current):
    a = Kp * (target - current)
    
    return a

def pi_2_pi(angle):
    return angle_mod(angle)


def solve_DARE(A, B, Q, R):
    '''
    slove a discrete time_Algebraic Riccati equation (DARE),
    求解P = Q + A.T @ P @ A - A.T @ P @ B @ la.inv(R + B.T @ P @ B) @ B.T @ P @ A,
    P初始值取Q, 经过迭代得到, 
    k = la.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
    u = -k @ x
    '''
    X = Q
    Xn = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        Xn = A.T @ X @ A - A.T @ X @ B @ \
            la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        if (abs(Xn - X)).max < eps:
            break
        X = Xn
    
    return Xn

def dlqr(A, B, Q, R):
    '''
    Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T @ Q @ x[k] + u[k].T @ R @ u[k]
    '''

    # 1st, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # 2nd, compute LQR gain
    K = la.inv(R + B.T @ X @ B) @ (B.T @ X @ A)

    # 3th, compute eigenvalue of matrix
    eigVals, eigvecs = la.eig(A - B * K)

    return K, X, eigVals # matlab tool, [K, S, E] = dlqr(A, B, Q, R)

def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt

    '''
        --------
        1 dt 0 0
    A   0 0  v 0
        0 0  1 dt
        0 0  0 0
        --------
    '''

    B = np.zeros((4, 1))
    B[0, 3] = v / L

    K, _, _ = dlqr(A, B, Q, R)

    x = np.zeros((4, 1))
    x[0, 0] = e
    x[0, 1] = (e - pe) / dt
    x[0, 2] = th_e
    x[0, 3] = (th_e - pth_e) / dt

    # 前馈控制
    ff = math.atan2(k * L, 1)
    # 反馈控制
    fb = pi_2_pi((-K @ x)[0, 0])

    # u = -k @ x + delta_f
    delta = ff + fb

    return delta, ind, e, th_e




def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    # 怎么判断state在参考路径点的前后
    angle = pi_2_pi(cyaw[ind] - math.atan(dyl, dxl))
    if angle < 0:
        mind *= 1.0

    return ind, mind


if __name__ == '__main__':
    pass