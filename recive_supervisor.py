""" Supervisor which runs the optimization """

"""
reciver listens to channel 5
emitter is on channel 4

Data is recived in the same order as it is sent
so the timings should be same
"""

import math
from cvxpy import *
import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')
from scipy.signal import cont2discrete , tf2ss
import time
import sys


from controller import Supervisor

## initialization
Ts = 0.05
TIME_STEP = int(1000*Ts)
supervisor = Supervisor()
receiver = supervisor.getDevice("receiver")
emitter = supervisor.getDevice("emitter")
receiver.enable(TIME_STEP)

accel = 0
brake = 0

## Parameters
T_eng =0.460
K_eng = 0.732
A_f = -1/T_eng
B_f = -K_eng/T_eng
C = np.eye(3)
T_hw = 1.3   ## also mentioned in controller
Ts = 0.1
T_total = 30
T = int(T_total/Ts)

# Discretize the system
A = np.array([[0, 1, -T_hw], [0, 0, -1], [0, 0, A_f]])
B = np.array([[0], [0], [B_f]])
sys2 = cont2discrete((A, B, np.eye(3), 0), Ts, method='zoh')
A, B, C, D , dt = sys2

(nx, nu) = B.shape

Q = np.array([[1 , 0, 0],[0,0,0],[0,0,0]])
R = np.eye(nu)*0
P = np.eye(nx)*0

umax = 3.5
umin = -3

xmin = np.array([[0] ,[-1.5]])  # state constraints
xmax = np.array([2])  # state constraints

N = 10  # horizon

previous_message = ''
xr = [0 , 0 , 0]

curr_time = 0
prev_time = 0

def rnd(number, precision=3):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)

def cvxpy_reverse1(A, B, N, Q, R, P, x0, xr, umax=None, umin=None, xmin=None, xmax=None):
    is_sol =  False
    (nx, nu) = B.shape

    # mpc calculation  x0 = distance, x1 = velocity, x2 = accel
    x = cvxpy.Variable((nx, N + 1))
    u = cvxpy.Variable((nu, N))
    weights = np.ones(5)
    w0 = weights[0]*15
    w1 = weights[1]*15
    w2 = weights[2]*3
    w3 = weights[3]*20
    w4 = weights[4]*0
    R = np.eye(nu)

    costlist = 0.0
    constraints = []

    for t in range(N):
        # costlist +=  cvxpy.quad_form(x[:, t] - xr[1], Q)
        costlist +=  (w1*((x[1, t]-xr[1])) + w2*(x[2, t]-xr[2]) + w0*((x[0, t]-xr[0]))) #cvxpy.quad_form(x[:, t]- xr, Q) #
        # costlist += cvxpy.quad_form(u[:, t], R)
        costlist += w3*u[:, t]

        constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t]]

        if xmin is not None:
            constraints += [x[0, t] >= xmin[0, 0]]  # state is greater than x min
            constraints += [x[2, t] >= xmin[1, 0]]
        if xmax is not None:
            constraints += [x[2, t] <= xmax[0]] # state is less than x max

    # costlist += 0.5 *((x[1, N]-xr[1]) + (x[2, N]-xr[2]) + (x[0, N]-xr[0])) # terminal cost #0.5 *((x[1, N]-xr[1])**2 + (x[2, N]-xr[2])**2 + (x[0, N]-xr[0])**2)  # terminal cost
    costlist += w4 *((x[1, N]-xr[1]) + (x[2, N]-xr[2])**2 + (x[0, N]-xr[0]))
    # costlist +=  cvxpy.quad_form(x[:, N] - xr[1], P)
    if xmin is not None:
        constraints += [x[0, t] >= xmin[0, 0]]  # state is greater than x min
        constraints += [x[2, t] >= xmin[1, 0]]
    if xmax is not None:
        constraints += [x[2, t] <= xmax[0]] # state is less than x max

    if umax is not None:
        constraints += [u <= umax]  # input constraints
    if umin is not None:
        constraints += [u >= umin]  # input constraints

    ur1_max = 2
    ur1_min = -2
    # add input rate constraints
    # rate of change of u1 constraint
    for i in range(N-1):
        constraints += [u[0,i+1] - u[0,i] <= ur1_max]
        constraints += [u[0,i+1] - u[0,i] >= ur1_min]

    # print("this is inside functio",x0)
    constraints += [x[:, 0] == x0]  # inital state constraints

    prob = cvxpy.Problem(cvxpy.Minimize(costlist), constraints)

    prob.solve(solver=ECOS , warm_start=True) #solver= OSQP,

    if prob.status == 'optimal':
    # Problem was solved successfully
        is_sol = True
        print("Optimal solution found!")
    elif prob.status == 'infeasible':
        # Problem is infeasible (no feasible solution exists)
        print("Problem is infeasible.")
    elif prob.status == 'unbounded':
        # Problem is unbounded (no finite optimal solution exists)
        print("Problem is unbounded.")
    else:
        # Solver failed or terminated prematurely
        print("Solver failed or terminated prematurely. Status:", prob.status)

    return is_sol , x.value, u.value, costlist.value


while supervisor.step(TIME_STEP) != -1:
    # recive the data
    # print("this is sup Q length",receiver.getQueueLength())
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        x0 = rnd(np.frombuffer(message, dtype=np.float64))
        print(f"this is x0 from supervisor  {x0} ")
        receiver.nextPacket()

    # give to optimization
        succ, x, u, c = cvxpy_reverse1(A, B, N, Q, R, P, x0, xr, umax = umax, umin = umin, xmin = xmin, xmax = xmax)
        if succ:
            x0 = rnd(x[:,0])

    # make the command
        if succ:
            if  u[0,0] >=0:
                accel = np.nan
                brake = u[0,0]/3.5
            else:
                brake = np.nan
                accel = u[0,0]/-3
        else:
            brake = 10
            accel = np.nan

    command = rnd(np.array([accel, brake]))
    message = command.tobytes()
    # send the commands
    message_size = sys.getsizeof(message)
    # print(message_size)

    if message != '' : #and message != previous_message:
        previous_message = message
        emitter.send(message)

    curr_time = time.time()
    # print("time taken is", curr_time - prev_time)
    prev_time = time.time()
